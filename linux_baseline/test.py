import os

# Files to skip explicitly
skip_files = [
    "test.py",
    ".cproject",
    "combined_output.txt",
    "empty_ref_1100x720.raw",
    "ReadMe.txt",
    "roi_map_output.json"
]

folder = os.path.dirname(os.path.abspath(__file__))
script_name = os.path.basename(__file__)
output_file = "combined_output.txt"

with open(os.path.join(folder, output_file), "w", encoding="utf-8") as out:
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)

        # Skip folders, this script, output file, and anything in skip_files
        if not os.path.isfile(file_path):
            continue
        if filename == script_name:
            continue
        if filename in skip_files:
            continue

        # Only include .c, .h, and Makefile
        if not (filename.endswith(".c") or filename.endswith(".h") or filename == "Makefile"):
            continue

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            out.write(f"{filename}:\n")
            out.write(content)
            out.write("\n\n")
        except Exception:
            pass

print(f"{output_file} created successfully.")