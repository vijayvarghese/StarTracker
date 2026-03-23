# import csv
# import argparse
# import math
# from tqdm import tqdm




# INPUT_FILE = "../data/processed/hipparcos_mag65.csv"
# OUTPUT_FILE = "../data/processed/hipparcos_mag65_lookup.csv"




# from astropy.coordinates import SkyCoord
# import astropy.units as u

# c1 = SkyCoord(ra=ra1_Rigel_deg*u.deg, dec=dec1_Rigel_deg*u.deg)
# c2 = SkyCoord(ra=ra2_Betelgeuse_deg*u.deg, dec=dec2_Betelgeuse_deg*u.deg)
# sep = c1.separation(c2)

# print(f"Separation: {sep.degree} degrees")





# with open(INPUT_FILE, 'r', newline='', encoding="utf-8") as fin, \
#      open(OUTPUT_FILE, "w", newline='', encoding="utf-8") as fout:

#     reader = csv.DictReader(fin)
#     writer = csv.writer(fout)

#     # Write header
#     writer.writerow(["id", "star1", "star2", "ang_sep"])

#     rows = []

#     for row in reader:
#         rows.append(row)

#     n = len(rows)-1

#     total = n * (n - 1) // 2

#     print(total)


#     with tqdm(total=total, desc="Processing pairs") as pbar:
#         for i in range(n):
#             for j in range(i + 1, n):
#                 ra1 = float(rows[i]["ra_deg"])
#                 dec1 = float(rows[i]["dec_deg"])
#                 id1 = rows[i]["id"].strip()

#                 ra2 = float(rows[j]["ra_deg"])
#                 dec2 = float(rows[j]["dec_deg"])
#                 id2 = rows[j]["id"].strip()

#                 #print(rows[i], rows[j])

#                 c1 = SkyCoord(ra=ra1*u.deg, dec=dec1*u.deg)
#                 c2 = SkyCoord(ra=ra2*u.deg, dec=dec2*u.deg)
#                 sep = c1.separation(c2)

#                 try:
#                     writer.writerow([
#                     f"{id1}",
#                     f"{id2}",
#                     f"{sep.degree:.6f}"
#                     ])
#                 except:
#                     print("Write Err")
#                     continue

#                 #print(id1,id2,sep.degree)
#                 pbar.update(1)



#-----------------------New fast Implimentation-------------------
import csv
from tqdm import tqdm
from astropy.coordinates import SkyCoord
import astropy.units as u

INPUT_FILE = "../data/processed/6.5/hipparcos_mag65.csv"
OUTPUT_FILE = "../data/processed/6.5/hipparcos_mag65_lookup_fov23.csv"

# -------------------------------------------------
# 1) Load CSV once
# -------------------------------------------------

rows = []
with open(INPUT_FILE, 'r', newline='', encoding="utf-8") as fin:
    reader = csv.DictReader(fin)
    for row in reader:
        rows.append(row)

ids = [r["id"].strip() for r in rows]
ra  = [float(r["ra_deg"]) for r in rows]
dec = [float(r["dec_deg"]) for r in rows]

n = len(rows)
total = n * (n - 1) // 2

print("Total pairs:", total)

# -------------------------------------------------
# 2) Build SkyCoord ONCE (big speedup)
# -------------------------------------------------

coords = SkyCoord(ra=ra * u.deg, dec=dec * u.deg)

# -------------------------------------------------
# 3) Vectorized separations + buffered writing
# -------------------------------------------------
FOV_MAX_DEG = 23.0     # star tracker FoV (upper bound)
ANGLE_MIN_DEG = 0.1   # lower bound (avoid duplicates / noise)
BATCH = 5000
buffer = []

with open(OUTPUT_FILE, "w", newline='', encoding="utf-8") as fout:
    writer = csv.writer(fout)
    writer.writerow(["star1", "star2", "ang_sep"])

    with tqdm(total=total, desc="Processing pairs") as pbar:
        for i in range(n - 1):
            seps = coords[i].separation(coords[i+1:])
            id1 = ids[i]
            

            for j, sep in enumerate(seps, start=i+1):

                if sep.degree < ANGLE_MIN_DEG:
                    continue

                if sep.degree > FOV_MAX_DEG:
                    continue

                buffer.append([id1, ids[j], f"{sep.degree:.6f}"])

                if len(buffer) >= BATCH:
                    writer.writerows(buffer)
                    buffer.clear()

            pbar.update(n - i - 1)

        if buffer:
            writer.writerows(buffer)

print("Done.")





#parser = argparse.ArgumentParser(description="hipparcos-voidmain CSV file angular seperation lookup maker (2point).")
#parser.add_argument("input_file", help="Path to the base CSV file")
#parser.add_argument("output_file", help="Path to save the new CSV")
#
#args = parser.parse_args()
#
#INPUT_FILE = args.input_file
#OUTPUT_FILE = args.output_file
#Rigel,78.6345,-8.2016,0.12
#Betelgeuse,88.7929,7.4071,0.42

#ra1_Rigel_deg = math.radians(78.6345)
#dec1_Rigel_deg = math.radians(-8.2016)
#
#ra1_Betelgeuse_deg = math.radians(88.7929)
#dec1_Betelgeuse_deg = math.radians(7.4071)

#ra1_Rigel_deg = 78.6345
#dec1_Rigel_deg = -8.2016
#ra2_Betelgeuse_deg = 88.7929
#dec2_Betelgeuse_deg = 7.4071

#def angular_separation_law_of_cosines(ra1_deg, dec1_deg, ra2_deg, dec2_deg):
#    #With Law of cosines
#
#    # Convert all inputs to radians
#    ra1, dec1 = math.radians(ra1_deg), math.radians(dec1_deg)
#    ra2, dec2 = math.radians(ra2_deg), math.radians(dec2_deg)
#    
#    # Haversine formula
#    d_ra = ra2 - ra1
#    d_dec = dec2 - dec1
#    
#    a = math.sin(d_dec / 2)**2 + math.cos(dec1) * math.cos(dec2) * math.sin(d_ra / 2)**2
#    theta_rad = 2 * math.asin(math.sqrt(a))
#    
#    # Convert result back to degrees
#    return math.degrees(theta_rad)


#print(angular_separation(ra1_Rigel_deg,dec1_Rigel_deg,ra2_Betelgeuse_deg,dec2_Betelgeuse_deg))



#def angular_separation_harversine(ra1_deg, dec1_deg, ra2_deg, dec2_deg):
