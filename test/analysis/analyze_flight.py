#!/usr/bin/env python3
"""
Interactive 3D trajectory plot from a flight log JSON file.

Features:
  - Time slider to scrub through trajectories
  - LED state shown per drone at the selected time
  - Drone 0 labeled as Main
  - DTW alignment analysis between each follower and the main drone
  - Waypoint success check against target_waypoints_times.json

Usage:
   python3 test/analysis/analyze_flight.py [path/to/log.json]
   python3 test/analysis/analyze_flight.py --latest
   python3 test/analysis/analyze_flight.py --latest --target test/analysis/config/target_waypoints_times.json
"""

import argparse
import json

import os
import sys

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, Slider
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

LOG_DIR = os.path.join(os.path.dirname(__file__), '..', 'logs')
TARGET_FILE = os.path.join(os.path.dirname(__file__), 'config', 'target_waypoints_mission_01.json')

DRONE_COLORS = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']


# ── helpers ──────────────────────────────────────────────────────────

def latest_file():
    files = sorted(
        [f for f in os.listdir(LOG_DIR) if f.endswith('.json')],
        reverse=True,
    )
    if not files:
        print(f'No log files found in {LOG_DIR}')
        sys.exit(1)
    return os.path.join(LOG_DIR, files[0])


def load_data(path):
    with open(path) as f:
        data = json.load(f)

    meta = data['meta']
    raw = data['data']

    print(f'  Drones: {meta["num_drones"]}')
    print(f'  Samples: {meta["num_samples"]}')

    all_times = []
    drones = {}
    for name, samples in raw.items():
        did = int(name.split('_')[1])
        traj = {'t': [], 'n': [], 'e': [], 'd': [], 'led': []}
        for s in samples:
            traj['t'].append(s['t'])
            traj['n'].append(s['n'])
            traj['e'].append(s['e'])
            traj['d'].append(s['d'])
            traj['led'].append(s['led'])
            all_times.append(s['t'])
        drones[did] = traj

    t0 = min(all_times)
    for traj in drones.values():
        traj['t'] = [t - t0 for t in traj['t']]

    return meta, drones


def load_targets(path):
    """Load target waypoints JSON with absolute ENU waypoints."""
    with open(path) as f:
        data = json.load(f)

    threshold = data['threshold_m']
    wpts = data['waypoints']

    ned_offsets = [(wp['n'], wp['e'], wp['d']) for wp in wpts]
    labels = [wp.get('label', str(i)) for i, wp in enumerate(wpts)]

    return {
        'ned_offsets': ned_offsets,
        'labels': labels,
        'threshold': threshold,
    }


# ── DTW ──────────────────────────────────────────────────────────────

def dtw_cost_matrix(x, y):
    """Compute DTW cost matrix between two NxD arrays."""
    m, n = len(x), len(y)
    cost = np.full((m + 1, n + 1), np.inf)
    cost[0, 0] = 0.0
    for i in range(1, m + 1):
        for j in range(1, n + 1):
            d = np.linalg.norm(x[i - 1] - y[j - 1])
            cost[i, j] = d + min(cost[i - 1, j], cost[i, j - 1], cost[i - 1, j - 1])
    return cost


def dtw_backtrack(cost):
    """Backtrack through cost matrix to find optimal path.
    Returns list of (i, j) indices (0-based)."""
    m, n = cost.shape
    i, j = m - 1, n - 1
    path = [(i - 1, j - 1)]
    while i > 1 or j > 1:
        if i == 1:
            j -= 1
        elif j == 1:
            i -= 1
        else:
            idx = np.argmin([cost[i - 1, j], cost[i, j - 1], cost[i - 1, j - 1]])
            if idx == 0:
                i -= 1
            elif idx == 1:
                j -= 1
            else:
                i -= 1
                j -= 1
        path.append((i - 1, j - 1))
    path.reverse()
    return np.array(path)


# ── waypoint check ───────────────────────────────────────────────────

def check_waypoints(drones, target_info):
    """For each follower drone, find closest approach to each waypoint.

    Returns:
        results: {drone_id: [(waypoint_idx, reached_bool, dist, target_pos, actual_pos), ...]}
    """
    offsets = target_info['ned_offsets']
    threshold = target_info['threshold']

    results = {}
    for did in sorted(drones):
        if did not in drones or did == 0:
            continue
        traj = drones[did]
        positions = np.column_stack([traj['n'], traj['e'], traj['d']])
        drone_results = []

        for wi, (wn, we, wd) in enumerate(offsets):
            target_pos = (float(wn), float(we), float(wd))
            dists = np.linalg.norm(positions - np.array([wn, we, wd]), axis=1)
            min_idx = np.argmin(dists)
            min_dist = dists[min_idx]
            actual_pos = (float(positions[min_idx][0]),
                          float(positions[min_idx][1]),
                          float(positions[min_idx][2]))
            reached = float(min_dist) <= threshold
            drone_results.append((wi, reached, float(min_dist), target_pos, actual_pos))

        results[did] = drone_results

    return results


# ── Metrics reporting ────────────────────────────────────────────────

def compute_and_report_metrics(drones, target_info=None, log_path=None):
    """Compute DTW and waypoint metrics, print to console, save to JSON."""
    import datetime

    d0 = drones[0]
    pos0 = np.column_stack([d0['n'], d0['e'], d0['d']])
    followers = [did for did in sorted(drones) if did != 0]

    results = {
        'log_file': log_path or '',
        'num_drones': len(drones),
        'followers': [],
        'waypoint_success': None,
    }

    # ── DTW per follower ──
    for did in followers:
        traj = drones[did]
        pos_f = np.column_stack([traj['n'], traj['e'], traj['d']])

        step = max(1, len(pos0) // 500)
        p0 = pos0[::step]
        pf = pos_f[::step]
        t0_arr = np.array(d0['t'][::step])
        tf_arr = np.array(traj['t'][::step])

        cost = dtw_cost_matrix(p0, pf)
        path = dtw_backtrack(cost)
        leader_t = t0_arr[path[:, 0]]
        follower_t = tf_arr[path[:, 1]]
        shift = leader_t - follower_t
        mean_err = cost[-1, -1] / len(path)

        entry = {
            'drone_id': did,
            'dtw_total_cost': round(float(cost[-1, -1]), 2),
            'dtw_aligned_points': len(path),
            'dtw_mean_error_per_step_m': round(float(mean_err), 4),
            'time_shift_mean_s': round(float(np.mean(shift)), 3),
            'time_shift_std_s': round(float(np.std(shift)), 3),
            'time_shift_min_s': round(float(np.min(shift)), 3),
            'time_shift_max_s': round(float(np.max(shift)), 3),
        }
        results['followers'].append(entry)

    # ── Console output ──
    print('\n' + '=' * 65)
    print('DTW ANALYSIS')
    print('=' * 65)
    for e in results['followers']:
        print(f'  Drone {e["drone_id"]}:')
        print(f'    Total cost:        {e["dtw_total_cost"]:.1f}')
        print(f'    Aligned points:    {e["dtw_aligned_points"]}')
        print(f'    Mean error/step:   {e["dtw_mean_error_per_step_m"]:.4f} m')
        print(f'    Time shift:        {e["time_shift_mean_s"]:.2f} ± {e["time_shift_std_s"]:.2f} s'
              f'  [{e["time_shift_min_s"]:.1f}, {e["time_shift_max_s"]:.1f}]')

    # ── Waypoint success ──
    if target_info:
        wp_results = check_waypoints(drones, target_info)
        results['waypoint_success'] = wp_entries = []
        print('\n' + '-' * 65)
        print('WAYPOINT SUCCESS')
        print('-' * 65)

        all_dids = sorted(wp_results)
        n_total = 0
        n_reached = 0
        for did in all_dids:
            drone_reached = 0
            drone_total = 0
            for wi, reached, dist, target_pos, actual_pos in wp_results[did]:
                drone_total += 1
                drone_reached += 1 if reached else 0
                wp_entries.append({
                    'drone_id': did,
                    'waypoint': wi,
                    'label': target_info['labels'][wi] if wi < len(target_info['labels']) else str(wi),
                    'reached': bool(reached),
                    'distance_m': round(float(dist), 4) if dist is not None else None,
                    'target_ned': target_pos,
                    'actual_ned': actual_pos,
                })
                mark = '+' if reached else '-'
                dist_str = f'{dist:.4f}' if dist is not None else '  N/A  '
                target_str = f'target=({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f})'
                actual_str = f'actual=({actual_pos[0]:.1f}, {actual_pos[1]:.1f}, {actual_pos[2]:.1f})'
                print(f'  Drone {did} {target_info["labels"][wi]:4s}: '
                      f'  {mark}  dist={dist_str}  {target_str}  {actual_str}')
            n_total += drone_total
            n_reached += drone_reached
        pct = 100 * n_reached / n_total if n_total else 0
        results['waypoint_summary'] = {
            'total': n_total,
            'reached': n_reached,
            'percentage': round(pct, 1),
        }
        print(f'\n  Total: {n_reached}/{n_total} ({pct:.0f}%)')

    # ── Save to JSON ──
    results_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
    os.makedirs(results_dir, exist_ok=True)

    if log_path:
        base = os.path.splitext(os.path.basename(log_path))[0]
    else:
        base = 'analysis'
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(results_dir, f'{base}_metrics_{ts}.json')
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'\nSaved metrics to {out_path}')
    print('=' * 65 + '\n')

    return results


# ── DTW figure ───────────────────────────────────────────────────────

def show_dtw_figure(drones, target_info=None):
    """Open one figure per plot type: cost matrices, time alignments,
    per-axis comparisons, and waypoint success."""
    d0 = drones[0]
    pos0 = np.column_stack([d0['n'], d0['e'], d0['d']])
    followers = [did for did in sorted(drones) if did != 0]

    waypoint_results = {}
    if target_info:
        waypoint_results = check_waypoints(drones, target_info)

    # Compute DTW for all followers first
    dtw_data = []
    for did in followers:
        color = DRONE_COLORS[did % len(DRONE_COLORS)]
        traj = drones[did]
        pos_f = np.column_stack([traj['n'], traj['e'], traj['d']])

        step = max(1, len(pos0) // 500)
        p0 = pos0[::step]
        pf = pos_f[::step]
        t0 = np.array(d0['t'][::step])
        tf = np.array(traj['t'][::step])

        cost = dtw_cost_matrix(p0, pf)
        path = dtw_backtrack(cost)
        leader_t = t0[path[:, 0]]
        follower_t = tf[path[:, 1]]
        shift = leader_t - follower_t
        warped_pf = pf[path[:, 1]]
        warped_t = t0[path[:, 0]]

        dtw_data.append({
            'did': did, 'color': color, 'p0': p0, 'pf': pf,
            't0': t0, 'tf': tf, 'cost': cost, 'path': path,
            'leader_t': leader_t, 'follower_t': follower_t,
            'shift': shift, 'warped_pf': warped_pf, 'warped_t': warped_t,
        })

    # ── Window 1: Cost matrices ──
    n_f = len(followers)
    fig1, axes1 = plt.subplots(n_f, 1, figsize=(7, 6 * n_f),
                               constrained_layout=True)
    if n_f == 1:
        axes1 = [axes1]
    fig1.suptitle('DTW Cost Matrices', fontsize=14)
    for i, dd in enumerate(dtw_data):
        ax = axes1[i]
        im = ax.imshow(dd['cost'][1:, 1:], origin='lower', aspect='auto',
                       cmap='viridis', interpolation='nearest')
        ax.plot(dd['path'][:, 1], dd['path'][:, 0], color='white',
                linewidth=0.8, alpha=0.7)
        ax.set_xlabel(f'Drone {dd["did"]} sample')
        ax.set_ylabel('Main sample')
        mean_err = dd["cost"][-1, -1] / len(dd["path"])
        ax.set_title(f'Drone {dd["did"]} — cost={dd["cost"][-1, -1]:.1f}, '
                     f'mean={mean_err:.2f}m/step')
        plt.colorbar(im, ax=ax, shrink=0.7)

    # ── Window 2: Time alignments ──
    fig2, axes2 = plt.subplots(n_f, 1, figsize=(7, 6 * n_f),
                               constrained_layout=True)
    if n_f == 1:
        axes2 = [axes2]
    fig2.suptitle('DTW Time Alignments', fontsize=14)
    for i, dd in enumerate(dtw_data):
        ax = axes2[i]
        ax.plot(dd['follower_t'], dd['leader_t'], color=dd['color'],
                linewidth=1.0, label=f'Drone {dd["did"]} → Main')
        ax.plot(dd['follower_t'], dd['follower_t'], '--', color='gray',
                alpha=0.5, label='Ideal')
        ax.fill_between(dd['follower_t'], dd['follower_t'], dd['leader_t'],
                        alpha=0.15, color=dd['color'])
        ax.set_xlabel('Follower time (s)')
        ax.set_ylabel('Leader time (s)')
        mean_shift = np.mean(dd["shift"])
        std_shift = np.std(dd["shift"])
        ax.set_title(f'Drone {dd["did"]} — shift = {mean_shift:.1f} ± {std_shift:.1f}s')
        ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.02, 1))
        ax.grid(True, alpha=0.3)

    # ── Window 3: Per-axis comparisons ──
    fig3, axes3 = plt.subplots(n_f, 1, figsize=(10, 6 * n_f),
                               constrained_layout=True)
    if n_f == 1:
        axes3 = [axes3]
    fig3.suptitle('Per-Axis Comparison (original vs DTW-warped)', fontsize=14)
    for i, dd in enumerate(dtw_data):
        ax = axes3[i]
        p0 = dd['p0']; pf = dd['pf']; t0 = dd['t0']; tf = dd['tf']
        warped_pf = dd['warped_pf']; warped_t = dd['warped_t']
        for ai in range(3):
            alpha_main = 0.8 if ai == 0 else 0.35
            lw_main = 1.5 if ai == 0 else 1.0
            ax.plot(t0, p0[:, ai], 'k', linewidth=lw_main, alpha=alpha_main,
                    label=f'Main {["N","E","D"][ai]}')
            ax.plot(tf, pf[:, ai], color=dd['color'], linestyle='--',
                    linewidth=1.0, alpha=0.4,
                    label=f'Drone {dd["did"]} {["N","E","D"][ai]} orig')
            ax.plot(warped_t, warped_pf[:, ai], color=dd['color'],
                    linewidth=1.5, alpha=0.9,
                    label=f'Drone {dd["did"]} {["N","E","D"][ai]} warped')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.set_title(f'Drone {dd["did"]}')
        ax.legend(fontsize=6, ncol=3, loc='upper left', bbox_to_anchor=(1.02, 1))
        ax.grid(True, alpha=0.3)

    # ── Window 4: Success matrix ──
    all_dids = [dd['did'] for dd in dtw_data]
    if waypoint_results:
        n_wpts = len(target_info['ned_offsets'])
        n_f = len(all_dids)

        success_matrix = np.zeros((n_f, n_wpts))
        for ri, did in enumerate(all_dids):
            if did not in waypoint_results:
                continue
            for wi, reached, *_unused in waypoint_results[did]:
                success_matrix[ri, wi] = 1.0 if reached else 0.0

        fig4 = plt.figure(figsize=(10, 3 + 0.4 * n_f))
        fig4.suptitle('Waypoint Success Matrix', fontsize=14)
        ax_success = fig4.add_subplot()
        im2 = ax_success.imshow(success_matrix, cmap='RdYlGn', vmin=0, vmax=1,
                                aspect='auto', interpolation='nearest')
        plt.colorbar(im2, ax=ax_success, shrink=0.7, label='Reached')
        ax_success.set_xlabel('Waypoint index')
        ax_success.set_ylabel('Follower drone')
        ax_success.set_yticks(range(n_f))
        ax_success.set_yticklabels([f'Drone {did}' for did in all_dids])
        ax_success.set_xticks(range(n_wpts))
        wpt_labels = target_info.get('labels', [str(i) for i in range(n_wpts)])
        ax_success.set_xticklabels(wpt_labels, fontsize=7, rotation=45)

        for ri in range(n_f):
            for ci in range(n_wpts):
                val = success_matrix[ri, ci]
                ax_success.text(ci, ri, '✓' if val else '✗',
                               ha='center', va='center',
                               fontsize=10, color='black' if val > 0.5 else 'white',
                               fontweight='bold')

        total_possible = n_f * n_wpts
        total_reached = int(success_matrix.sum())
        ax_success.text(0.5, -0.25,
                       f'Total: {total_reached}/{total_possible} '
                       f'({100 * total_reached / total_possible:.0f}%)',
                       transform=ax_success.transAxes,
                       ha='center', fontsize=10,
                       bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))

        # ── Window 5: Details table ──
        col_labels = ['Drone', 'WP', 'Reached', 'Dist (m)', 'Target NED', 'Actual NED']
        rows = []
        for did in all_dids:
            if did not in waypoint_results:
                continue
            for wi, reached, dist, target_pos, actual_pos in waypoint_results[did]:
                dist_str = f'{dist:.3f}' if dist is not None else '-'
                tgt_str = f'({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f})'
                act_str = f'({actual_pos[0]:.1f}, {actual_pos[1]:.1f}, {actual_pos[2]:.1f})'
                rows.append([
                    f'{did}',
                    target_info['labels'][wi],
                    '✓' if reached else '✗',
                    dist_str,
                    tgt_str,
                    act_str,
                ])
        if rows:
            n_rows = len(rows)
            fig5 = plt.figure(figsize=(16, 2 + 0.45 * n_rows))
            fig5.suptitle('Waypoint Details', fontsize=14)
            ax_details = fig5.add_subplot()
            ax_details.axis('off')
            table = ax_details.table(cellText=rows, colLabels=col_labels,
                                    loc='center', cellLoc='center',
                                    bbox=[0, 0, 1, 1])
            table.auto_set_font_size(False)
            table.set_fontsize(9)
            for ri, row_data in enumerate(rows):
                reached = row_data[2] == '✓'
                clr = '#d4edda' if reached else '#f8d7da'
                for ci in range(len(col_labels)):
                    table[(ri + 1, ci)].set_facecolor(clr)
            fig5.subplots_adjust(top=0.94, left=0.05, right=0.95)

    plt.show()


# ── 3D trajectory figure ────────────────────────────────────────────

def build_legend(ax, drones, time_idx):
    handles = []
    for did, traj in sorted(drones.items()):
        color = DRONE_COLORS[did % len(DRONE_COLORS)]
        idx = min(time_idx, len(traj['led']) - 1) if time_idx < len(traj['led']) else 0
        led = traj['led'][idx] if traj['led'] else ''
        label = f'Drone {did}'
        if did == 0:
            label += ' (Main)'
        if led:
            label += f' — {led}'
        h = Line2D([0], [0], color=color, lw=2, label=label)
        handles.append(h)

    handles.append(Line2D([0], [0], color='none', label=''))
    handles.append(Line2D([0], [0], marker='o', color='gray',
                          markerfacecolor='gray', markersize=8, label='Start'))
    handles.append(Line2D([0], [0], marker='s', color='gray',
                          markerfacecolor='gray', markersize=8, label='End'))

    ax.legend(handles=handles, loc='upper left', title='Drones', fontsize=9)

    return handles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('path', nargs='?', help='Path to log JSON')
    parser.add_argument('--latest', action='store_true', help='Use latest log')
    parser.add_argument('--target', default=None,
                        help=f'Path to target waypoints JSON (default: {TARGET_FILE})')
    parser.add_argument('--no-dtw', action='store_true',
                        help='Skip DTW analysis window')
    args = parser.parse_args()

    if args.latest or not args.path:
        path = latest_file()
    else:
        path = args.path

    print(f'Loading {path}...')
    meta, drones = load_data(path)

    t_min = 0.0
    t_max = max(t for traj in drones.values() for t in traj['t'])

    # ── Load target file ──
    target_info = None
    if not args.no_dtw:
        target_path = args.target or TARGET_FILE
        if os.path.exists(target_path):
            print(f'Loading targets from {target_path}')
            target_info = load_targets(target_path)
        else:
            print(f'Target file not found at {target_path}, skipping waypoint checks')

    # ── 3D trajectory ──
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(1, 1, bottom=0.12)
    ax = fig.add_subplot(gs[0], projection='3d')

    lines = {}
    start_scatters = {}
    end_scatters = {}

    for did, traj in sorted(drones.items()):
        n = np.array(traj['n'])
        e = np.array(traj['e'])
        d = np.array(traj['d'])
        color = DRONE_COLORS[did % len(DRONE_COLORS)]
        lines[did], = ax.plot([], [], [], color=color, linewidth=1.5)
        start_scatters[did] = ax.scatter([], [], [], color=color, marker='o',
                                         s=80, edgecolors='black', zorder=5)
        end_scatters[did] = ax.scatter([], [], [], color=color, marker='s',
                                       s=80, edgecolors='black', zorder=5)

    all_n = np.concatenate([np.array(t['n']) for t in drones.values()])
    all_e = np.concatenate([np.array(t['e']) for t in drones.values()])
    all_d = np.concatenate([np.array(t['d']) for t in drones.values()])

    padding = 5.0
    n_lim = (all_n.min() - padding, all_n.max() + padding)
    e_lim = (all_e.min() - padding, all_e.max() + padding)
    d_lim = (all_d.min() - padding, all_d.max() + padding)

    ax.set_xlabel('North (m)')
    ax.set_ylabel('East (m)')
    ax.set_zlabel('Down (m)')
    ax.set_title(f'Drone Trajectories — {meta["start_time"]}')
    ax.set_xlim(n_lim)
    ax.set_ylim(e_lim)
    ax.set_zlim(d_lim)
    ax.view_init(elev=25, azim=-45)
    ax.invert_zaxis()

    # ── Plot waypoint target positions as reference ──
    waypoint_markers = {}
    if target_info:
        offsets = target_info['ned_offsets']
        wpt_n = [o[0] for o in offsets]
        wpt_e = [o[1] for o in offsets]
        wpt_d = [o[2] for o in offsets]
        # Only plot waypoints if they're within the visible limits
        ax.scatter(wpt_n, wpt_e, wpt_d, marker='x', color='black', s=40,
                   alpha=0.4, zorder=1, label='Target waypoints')

    max_idx = max(len(t['n']) - 1 for t in drones.values())
    build_legend(ax, drones, max_idx)

    ax_color = 'lightgoldenrodyellow'
    slider_ax = fig.add_axes([0.2, 0.03, 0.6, 0.03], facecolor=ax_color)
    step = max(
        np.median(np.diff(traj['t'])) if len(traj['t']) > 1 else 0.1
        for traj in drones.values()
    )
    if step <= 0:
        step = 0.1
    slider = Slider(slider_ax, 'Time (s)', t_min, t_max,
                    valinit=t_min, valstep=step)

    btn_left_ax = fig.add_axes([0.06, 0.03, 0.06, 0.03])
    btn_right_ax = fig.add_axes([0.88, 0.03, 0.06, 0.03])
    btn_left = Button(btn_left_ax, '◀')
    btn_right = Button(btn_right_ax, '▶')

    def on_left(_):
        slider.set_val(max(t_min, slider.val - step))

    def on_right(_):
        slider.set_val(min(t_max, slider.val + step))

    btn_left.on_clicked(on_left)
    btn_right.on_clicked(on_right)

    def find_idx(traj, t):
        arr = np.array(traj['t'])
        idx = np.searchsorted(arr, t, side='right') - 1
        return max(0, idx)

    def update(val):
        t = slider.val
        for did, traj in drones.items():
            idx = find_idx(traj, t)
            n = np.array(traj['n'][:idx + 1])
            e = np.array(traj['e'][:idx + 1])
            d = np.array(traj['d'][:idx + 1])
            lines[did].set_data_3d(n, e, d)
            if idx >= 0 and len(n) > 0:
                start_scatters[did].set_offsets([(n[0], e[0])])
                start_scatters[did].set_3d_properties(d[0], zdir='z')
                end_scatters[did].set_offsets([(n[-1], e[-1])])
                end_scatters[did].set_3d_properties(d[-1], zdir='z')
        ax.set_title(f'Drone Trajectories — {meta["start_time"]}  (t = {t:.1f}s)')
        build_legend(ax, drones, find_idx(drones[0], t))
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(t_min)

    # ── DTW + waypoint success figure ──
    if not args.no_dtw and len(drones) > 1:
        show_dtw_figure(drones, target_info)

    # ── Report & save metrics ──
    if not args.no_dtw and len(drones) > 1:
        compute_and_report_metrics(drones, target_info, log_path=path)

    plt.show()


if __name__ == '__main__':
    main()
