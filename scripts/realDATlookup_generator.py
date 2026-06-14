import csv
import argparse

parser = argparse.ArgumentParser(description="hipparcos-voidmain CSV file subset maker w.r.t MAG.")
# Define positional arguments in the order they should be provided
parser.add_argument("Mag_Trimmed_csv", help="Path to the base CSV file")
parser.add_argument("Bin_Lookup_CSV", help="Path to  the Bin lookup CSV file")
parser.add_argument("output_file", help="Path to save the new CSV")



args = parser.parse_args()

print(f"Reading from: {args.Mag_Trimmed_csv}")
print(f"Bin lookup CSV file: {args.Bin_Lookup_CSV}")
print(f"Saving to: {args.output_file}")




#INPUT_FILE = "hipparcos-voidmain.csv"
#OUTPUT_FILE = "hipparcos_mag65.csv"
#MAG_LIMIT = 6.5


INPUT_FILE = args.Mag_Trimmed_csv
OUTPUT_FILE = args.output_file
ANGSEP_FILE = args.Bin_Lookup_CSV

count_angsep = 0
count_out = 0

with open(INPUT_FILE, newline='', encoding="utf-8") as fin, \
     open(ANGSEP_FILE, newline='', encoding="utf-8") as ain, \
     open(OUTPUT_FILE, "w", newline='', encoding="utf-8") as fout:
    
    #populating the dict with mag trimmed catelog
    mag_trimmed = dict()
    
    reader = csv.DictReader(fin)
    count_r = 0
    for row in reader:
        id = row["id"].strip()
        ra_deg = float(row["ra_deg"])
        dec_deg = float(row["dec_deg"])
        mag = float(row["mag"]) 
        mag_trimmed[id] = [ra_deg,dec_deg,mag]
        count_r += 1
    print("Populated OG Mag trimmed catelog : ",count_r)

    writer = csv.writer(fout)
    reader_AngSep = csv.DictReader(ain)
    # Write header
    writer.writerow(["id", "ra_deg", "dec_deg", "mag"])

    print("Iterate through ang_sep lookup:", ANGSEP_FILE)
    #iterate through ang_sep lookup
    Output = list()
    for row in reader_AngSep:
        count_angsep += 1
        hip = str()
        ra = float()
        dec = float()
        vmag = float()
        try:
            star1 = row["star1"].strip()
            star2 = row["star2"].strip()
            ang_sep = float(row["ang_sep"])


            #check in  mag_trimmed ! 
            if (not((len(mag_trimmed[star1]) > 0 or len(mag_trimmed[star1]) > 0))):
                continue
            if (len(mag_trimmed[star1]) > 0 and Output.count(star1)==0):
                Output.append(star1)
                writer.writerow([
                    f"{star1}",
                    f"{mag_trimmed[star1][0]:.6f}",
                    f"{mag_trimmed[star1][1]:.6f}",
                    f"{mag_trimmed[star1][2]:.2f}"
                ])
                count_out += 1
            if (len(mag_trimmed[star2]) > 0 and Output.count(star2)==0):
                Output.append(star2)
                writer.writerow([
                    f"{star2}",
                    f"{mag_trimmed[star2][0]:.6f}",
                    f"{mag_trimmed[star2][1]:.6f}",
                    f"{mag_trimmed[star2][2]:.2f}"
                ])
                count_out += 1

        except (ValueError, KeyError):
            # Skip malformed rows
            continue

print(f"Total rows read (OG Mag trimmed)     : {count_r}")
print(f"Total rows read (Ang Sep)     : {count_angsep}")
print(f"Stars written : {count_out}")
print(f"Output file         : {OUTPUT_FILE}")
print(f"Output file (count)         : {len(Output)}")


