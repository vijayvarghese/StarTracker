import csv

INPUT_FILE = "hipparcos-voidmain.csv"
OUTPUT_FILE = "hipparcos_mag65.csv"

MAG_LIMIT = 6.5

count_in = 0
count_out = 0

with open(INPUT_FILE, newline='', encoding="utf-8") as fin, \
     open(OUTPUT_FILE, "w", newline='', encoding="utf-8") as fout:

    reader = csv.DictReader(fin)
    writer = csv.writer(fout)

    # Write header
    writer.writerow(["id", "ra_deg", "dec_deg", "mag"])

    for row in reader:
        count_in += 1

        try:
            vmag = float(row["Vmag"])
            ra = float(row["RAdeg"])
            dec = float(row["DEdeg"])
            hip = row["HIP"].strip()

            # Filter by magnitude
            if vmag > MAG_LIMIT:
                continue

            # Basic sanity checks
            if not (0.0 <= ra < 360.0):
                continue
            if not (-90.0 <= dec <= 90.0):
                continue

            writer.writerow([
                f"HIP_{hip}",
                f"{ra:.6f}",
                f"{dec:.6f}",
                f"{vmag:.2f}"
            ])

            count_out += 1

        except (ValueError, KeyError):
            # Skip malformed rows
            continue

print(f"Total rows read     : {count_in}")
print(f"Stars written (≤6.5): {count_out}")
print(f"Output file         : {OUTPUT_FILE}")

