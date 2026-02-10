import sqlite3
import struct
import os
import numpy as np
import matplotlib.pyplot as plt
import utm as utm_lib

DB3_STATIONARY_OPEN = "Data/Open/stationary_open1_0.db3"
DB3_STATIONARY_OCCLUDED = "Data/Occluded/stationary_occluded_0.db3"
DB3_WALKING = "Data/Walking/walking_0.db3"

# Known positions
KNOWN_OPEN_LAT = 42.344
KNOWN_OPEN_LON = -71.0845
KNOWN_OCCLUDED_LAT = 42.345
KNOWN_OCCLUDED_LON = -71.0845

def parse_gpsmsg_cdr(raw_bytes):
    data = bytes(raw_bytes)

    off = 4

    sec = struct.unpack_from('<i', data, off)[0]
    off += 4

    nanosec = struct.unpack_from('<I', data, off)[0]
    off += 4

    str_len = struct.unpack_from('<I', data, off)[0]
    off += 4
    frame_id = data[off:off + str_len - 1].decode('ascii', errors='replace')
    off += str_len

    # Align to 8b boundar for float
    data_off = off - 4
    pad = (8 - (data_off % 8)) % 8
    off += pad

    lat, lon, alt, hdop, e, n, utc = struct.unpack_from('<7d', data, off)
    off += 7 * 8

    zone = struct.unpack_from('<i', data, off)[0]
    off += 4

    str_len2 = struct.unpack_from('<I', data, off)[0]
    off += 4
    letter = data[off:off + str_len2 - 1].decode('ascii', errors='replace')

    return {
        'sec': sec,
        'nanosec': nanosec,
        'frame_id': frame_id,
        'latitude': lat,
        'longitude': lon,
        'altitude': alt,
        'hdop': hdop,
        'utm_easting': e,
        'utm_northing': n,
        'utc': utc,
        'zone': zone,
        'letter': letter,
    }


def read_db3(db3_path):

    if not os.path.exists(db3_path):
        raise FileNotFoundError(f"File not found: {db3_path}")

    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    # Find GPS topic ID
    cursor.execute("SELECT id FROM topics WHERE name LIKE '%gps%'")
    row = cursor.fetchone()
    if row is None:
        raise ValueError(f"No GPS topic found in {db3_path}")
    topic_id = row[0]

    # Read all messages ordered by timestamp
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,)
    )
    rows = cursor.fetchall()
    conn.close()

    lat, lon, alt, easting, northing, hdop, utc = [], [], [], [], [], [], []
    timestamps = []

    for ts, raw_data in rows:
        msg = parse_gpsmsg_cdr(raw_data)
        lat.append(msg['latitude'])
        lon.append(msg['longitude'])
        alt.append(msg['altitude'])
        easting.append(msg['utm_easting'])
        northing.append(msg['utm_northing'])
        hdop.append(msg['hdop'])
        utc.append(msg['utc'])
        timestamps.append(ts / 1e9)

    print(f"  Read {len(lat)} messages from {db3_path}")
    print(f"  First point: lat={lat[0]:.6f}, lon={lon[0]:.6f}, alt={alt[0]:.1f}, "
          f"hdop={hdop[0]}, zone={msg['zone']}{msg['letter']}")

    return {
        'lat': np.array(lat),
        'lon': np.array(lon),
        'alt': np.array(alt),
        'easting': np.array(easting),
        'northing': np.array(northing),
        'hdop': np.array(hdop),
        'utc': np.array(utc),
        'time': np.array(timestamps),
    }


def known_to_utm(lat, lon):
    result = utm_lib.from_latlon(lat, lon)
    return result[0], result[1]

#Analysis

def analyze_stationary(data, known_lat, known_lon, label):
    known_e, known_n = known_to_utm(known_lat, known_lon)

    easting = data['easting']
    northing = data['northing']
    alt = data['alt']
    time_rel = data['time'] - data['time'][0]
    hdop_vals = data['hdop']

    #Scatterplot: Northing vs Easting
    e_offset = easting - easting[0]
    n_offset = northing - northing[0]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(e_offset, n_offset, c='blue', s=10, alpha=0.6)
    ax.set_xlabel('Easting offset (m)')
    ax.set_ylabel('Northing offset (m)')
    ax.set_title(f'Stationary {label}: Northing vs Easting (offset from first point)')
    ax.set_aspect('equal')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(f'stationary_{label}_scatter.png', dpi=150)
    plt.show()

    #Pos error calc
    easting_err = easting - known_e
    northing_err = northing - known_n
    position_err = np.sqrt(easting_err**2 + northing_err**2)

    #Histograms
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    axes[0].hist(easting_err, bins=25, color='steelblue', edgecolor='black', alpha=0.7)
    axes[0].set_xlabel('Easting Error (m)')
    axes[0].set_ylabel('Count')
    axes[0].set_title(f'{label}: Easting Error from Known')
    axes[0].axvline(x=0, color='red', linestyle='--', label='Known')
    axes[0].legend()

    axes[1].hist(northing_err, bins=25, color='darkorange', edgecolor='black', alpha=0.7)
    axes[1].set_xlabel('Northing Error (m)')
    axes[1].set_ylabel('Count')
    axes[1].set_title(f'{label}: Northing Error from Known')
    axes[1].axvline(x=0, color='red', linestyle='--', label='Known')
    axes[1].legend()

    axes[2].hist(position_err, bins=25, color='green', edgecolor='black', alpha=0.7)
    axes[2].set_xlabel('Position Error (m)')
    axes[2].set_ylabel('Count')
    axes[2].set_title(f'{label}: Euclidean Position Error')

    plt.tight_layout()
    plt.savefig(f'stationary_{label}_histogram.png', dpi=150)
    plt.show()

    # Altitude vs Time
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(time_rel, alt, 'g.-', markersize=3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title(f'Stationary {label}: Altitude vs Time')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(f'stationary_{label}_altitude.png', dpi=150)
    plt.show()

    # Stats
    mean_hdop = np.mean(hdop_vals)
    mean_err = np.mean(position_err)
    std_err = np.std(position_err)
    rms_err = np.sqrt(np.mean(position_err**2))
    max_err = np.max(position_err)

    print(f"  STATIONARY {label.upper()}")
    print(f"  Number of points:       {len(easting)}")
    print(f"  Mean HDOP:              {mean_hdop:.2f}")
    print(f"  Mean position error:    {mean_err:.2f} m")
    print(f"  Std of position error:  {std_err:.2f} m")
    print(f"  RMS position error:     {rms_err:.2f} m")
    print(f"  Max position error:     {max_err:.2f} m")
    print(f"  Mean easting error:     {np.mean(easting_err):.2f} m")
    print(f"  Mean northing error:    {np.mean(northing_err):.2f} m")
    print(f"  Expected error (HDOP x ~3m): {mean_hdop * 3:.2f} m")

    return {
        'mean_error': mean_err,
        'rms_error': rms_err,
        'std_error': std_err,
        'max_error': max_err,
        'mean_hdop': mean_hdop,
    }


def analyze_walking(data):

    easting = data['easting']
    northing = data['northing']
    alt = data['alt']
    time_rel = data['time'] - data['time'][0]

    # f.p. offset
    e_offset = easting - easting[0]
    n_offset = northing - northing[0]

    # Best fit line
    coeffs = np.polyfit(e_offset, n_offset, 1)
    m, b = coeffs
    poly = np.poly1d(coeffs)
    e_fit = np.linspace(e_offset.min(), e_offset.max(), 100)
    n_fit = poly(e_fit)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter(e_offset, n_offset, c='blue', s=10, alpha=0.6, label='GPS data')
    ax.plot(e_fit, n_fit, 'r-', linewidth=2, label='Best fit line')
    ax.set_xlabel('Easting offset (m)')
    ax.set_ylabel('Northing offset (m)')
    ax.set_title('Walking: Northing vs Easting with Line of Best Fit')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig('walking_scatter.png', dpi=150)
    plt.show()

    # distance from each point to best-fit line
    distances = np.abs(m * e_offset - n_offset + b) / np.sqrt(m**2 + 1)

    mean_dist = np.mean(distances)
    std_dist = np.std(distances)
    rms_dist = np.sqrt(np.mean(distances**2))
    max_dist = np.max(distances)

    # Altitude vs Time
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(time_rel, alt, 'g.-', markersize=3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Walking: Altitude vs Time')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig('walking_altitude.png', dpi=150)
    plt.show()

    print(f"  WALKING DATA")
    print(f"  Number of points:                {len(easting)}")
    print(f"  Best fit slope (m):              {m:.4f}")
    print(f"  Best fit intercept (b):          {b:.4f} m")
    print(f"  Mean distance from best fit:     {mean_dist:.2f} m")
    print(f"  Std distance from best fit:      {std_dist:.2f} m")
    print(f"  RMS distance from best fit:      {rms_dist:.2f} m")
    print(f"  Max distance from best fit:      {max_dist:.2f} m")
    print(f"  Mean HDOP:                       {np.mean(data['hdop']):.2f}")

    return {
        'mean_dist': mean_dist,
        'rms_dist': rms_dist,
        'std_dist': std_dist,
        'max_dist': max_dist,
    }

if __name__ == '__main__':

    print("\nStationary open data:")
    data_open = read_db3(DB3_STATIONARY_OPEN)

    print("\nStationary occluded data:")
    data_occluded = read_db3(DB3_STATIONARY_OCCLUDED)

    print("\nWalking data:")
    data_walking = read_db3(DB3_WALKING)

    stats_open = analyze_stationary(data_open, KNOWN_OPEN_LAT, KNOWN_OPEN_LON, "Open")
    stats_occluded = analyze_stationary(data_occluded, KNOWN_OCCLUDED_LAT, KNOWN_OCCLUDED_LON, "Occluded")
    stats_walking = analyze_walking(data_walking)

    print("Successfully executed")