import numpy as np

# Parameters
width = 40   # pixels (2.0m / 0.05m)
height = 14  # pixels (0.7m / 0.05m)
cost_value = 127  # 50% cost

# Create the mask
mask = np.full((height, width), cost_value, dtype=np.uint8)

# Write to PGM file
with open("dump_keepout_mask.pgm", "w") as f:
    f.write("P2\n")
    f.write(f"{width} {height}\n")
    f.write("255\n")
    for row in mask:
        f.write(" ".join(str(val) for val in row) + "\n")

print("dump_keepout_mask.pgm generated!") 