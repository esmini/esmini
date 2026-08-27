# This script adjust the material properties of Wavefront OBJ (.obj) associated .mat files exported from Blender
# Operations:
# - set ambient = 0
# for use in esmini

import os
import argparse

class Material:
    def __init__(self, name):
        self.name = name
        self.Ns = 0.0
        self.Ka = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]
        self.Ks = [0.0, 0.0, 0.0]
        self.Ke = [0.0, 0.0, 0.0]
        self.Ni = 1.0
        self.d = 1.0
        self.illum = 2

    def __repr__(self):
        return (f"Material(name='{self.name}', "
                f"Ns={self.Ns}, Ka={self.Ka}, Kd={self.Kd}, "
                f"Ks={self.Ks}, Ke={self.Ke}, Ni={self.Ni}, "
                f"d={self.d}, illum={self.illum})")

def parse_mtl(filename):
    materials = []
    current_material = None

    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return materials

    with open(filename, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        parts = line.split()
        command = parts[0]

        if command == 'newmtl':
            if current_material:
                materials.append(current_material)
            current_material = Material(parts[1])
        elif current_material:
            if command == 'Ns':
                current_material.Ns = float(parts[1])
            elif command == 'Ka':
                current_material.Ka = [float(x) for x in parts[1:4]]
            elif command == 'Kd':
                current_material.Kd = [float(x) for x in parts[1:4]]
            elif command == 'Ks':
                current_material.Ks = [float(x) for x in parts[1:4]]
            elif command == 'Ke':
                current_material.Ke = [float(x) for x in parts[1:4]]
            elif command == 'Ni':
                current_material.Ni = float(parts[1])
            elif command == 'd':
                current_material.d = float(parts[1])
            elif command == 'illum':
                current_material.illum = int(parts[1])

    if current_material:
        materials.append(current_material)

    return materials

def write_mtl(filename, materials):
    """Writes a list of Material objects to an MTL file."""
    with open(filename, 'w') as f:
        f.write(f"# original file from Blender\n# all ambient (Ka) reset to 0.0, for use in esmini\n\n")
        for i, mat in enumerate(materials):
            if i > 0:
                f.write("\n")
            f.write(f"newmtl {mat.name}\n")
            f.write(f"Ns {mat.Ns:.6f}\n")
            f.write(f"Ka {mat.Ka[0]:.6f} {mat.Ka[1]:.6f} {mat.Ka[2]:.6f}\n")
            f.write(f"Kd {mat.Kd[0]:.6f} {mat.Kd[1]:.6f} {mat.Kd[2]:.6f}\n")
            f.write(f"Ks {mat.Ks[0]:.6f} {mat.Ks[1]:.6f} {mat.Ks[2]:.6f}\n")
            f.write(f"Ke {mat.Ke[0]:.6f} {mat.Ke[1]:.6f} {mat.Ke[2]:.6f}\n")
            f.write(f"Ni {mat.Ni:.6f}\n")
            f.write(f"d {mat.d:.6f}\n")
            f.write(f"illum {mat.illum}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Set ambient (Ka) material properties to 0.0 and save to a new file.')
    parser.add_argument('input_file', help='material file (.mtl)')
    args = parser.parse_args()

    # Define output filename
    stem, _ = os.path.splitext(args.input_file)
    outfile = stem + '_fixed.mat'

    # Parse materials from input file
    mats = parse_mtl(args.input_file)

    if mats:
        # Set ambient color to 0 for all materials
        for m in mats:
            m.Ka = [0.0, 0.0, 0.0]

        # Write the modified materials to the new file
        write_mtl(outfile, mats)

        # swap filename
        orig_filename = stem + '_original.mat'
        if not os.path.isfile(orig_filename):
            os.rename(args.input_file, orig_filename)
            print(f"Original file renamed to {orig_filename}")
        else:
            print(f"Original file replaced {orig_filename}")
            os.replace(args.input_file, orig_filename)
        os.rename(outfile, args.input_file)
        print(f"Processed {len(mats)} material(s). Updated {args.input_file}")
    elif os.path.exists(args.input_file):
        print(f"No materials found in {args.input_file}.")
