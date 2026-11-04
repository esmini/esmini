import argparse
from collections import Counter
import math
from pathlib import Path
import re
import sys

import matplotlib.pyplot as plt


PERMUTATION = re.compile(r"\bParameter permutation \d+/\d+\b", re.IGNORECASE)
PARAMETER = re.compile(
    r"^(?:\[[^\]]*\][ \t]*)*(?:\t| {2,})([^:\s][^:]*):[ \t]*(.*?)\s*$"
)


def read_values(filename, parameter):
    values = []
    in_parameters = False
    with filename.open(encoding="utf-8") as logfile:
        for line_number, line in enumerate(logfile, 1):
            if PERMUTATION.search(line):
                in_parameters = True
                continue
            if not in_parameters:
                continue
            match = PARAMETER.match(line)
            if not match:
                in_parameters = False
                continue
            if match.group(1).strip() != parameter:
                continue
            try:
                value = float(match.group(2))
            except ValueError:
                values.append(match.group(2))
                continue
            if not math.isfinite(value):
                raise ValueError(
                    "{}:{}: {} has a non-finite value".format(
                        filename, line_number, parameter
                    )
                )
            values.append(match.group(2))
    return values


def main():
    parser = argparse.ArgumentParser(
        description="Plot a parameter distribution from esmini permutation logs"
    )
    parser.add_argument("--dir", required=True, type=Path, help="logfile directory")
    parser.add_argument(
        "--file", required=True, help="substring to match in .txt filenames"
    )
    parser.add_argument(
        "--param", required=True, help="parameter name (case-sensitive)"
    )
    parser.add_argument(
        "--bin-size",
        type=float,
        default=1.0,
        help="numeric histogram bin width (default: 1; ignored for categories)",
    )
    args = parser.parse_args()

    if not math.isfinite(args.bin_size) or args.bin_size <= 0:
        parser.error("--bin-size must be a finite, positive number")
    if not args.dir.is_dir():
        parser.error("not a directory: {}".format(args.dir))
    try:
        files = sorted(
            path
            for path in args.dir.iterdir()
            if path.is_file()
            and path.suffix == ".txt"
            and path.name != "log.txt"
            and args.file in path.name
        )
        if not files:
            parser.error("no matching .txt files in {}".format(args.dir))
        values = []
        for filename in files:
            file_values = read_values(filename, args.param)
            if not file_values:
                print(
                    "Warning: {} not found in permutation blocks in {}".format(
                        args.param, filename
                    ),
                    file=sys.stderr,
                )
            values.extend(file_values)
    except (OSError, UnicodeError, ValueError) as error:
        parser.error(str(error))
    if not values:
        parser.error("no values found for {}".format(args.param))

    print("Plotting {} values from {} files".format(len(values), len(files)))
    try:
        numeric_values = [float(value) for value in values]
    except ValueError:
        counts = Counter(values)
        labels = sorted(counts)
        plt.bar(labels, [counts[label] for label in labels], edgecolor="black")
        plt.xticks(rotation=45, ha="right")
    else:
        start = math.floor(min(numeric_values) / args.bin_size) * args.bin_size
        bin_count = math.floor((max(numeric_values) - start) / args.bin_size) + 1
        bins = [start + i * args.bin_size for i in range(bin_count + 1)]
        plt.hist(numeric_values, bins=bins, edgecolor="black")
    plt.xlabel(args.param)
    plt.ylabel("Count")
    plt.title("{} distribution ({} samples)".format(args.param, len(values)))
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
