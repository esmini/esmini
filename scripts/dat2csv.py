from dat import *

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description="Read and print .dat file")

    parser.add_argument("--file", "-f", help="dat filename")
    parser.add_argument(
        "--extended", "-e", action="store_true", help="add road coordinates"
    )
    parser.add_argument(
        "--file_refs",
        "-r",
        action="store_true",
        help="include odr and model file references",
    )
    parser.add_argument(
        "--time_mode",
        "-m",
        choices=[
            "original",
            "min_step",
            "min_step_mixed",
            "custom_time_step",
            "custom_time_step_mixed",
        ],
        default="original",
        help="control timestamps in the csv.",
    )
    parser.add_argument(
        "-t",
        "--time_step",
        type=float,
        action="store",
        default=0.05,
        help="The time step to use for the fixed time steps(ms).",
    )

    # Execute the parse_args() method
    args = parser.parse_args()

    dat = DATFile(args.file)
    dat.save_csv(
        extended=True if args.extended else False,
        include_file_refs=True if args.file_refs else False,
        mode=args.time_mode,
        step_time=args.time_step,
    )
    print(
        "Created "
        + os.path.splitext(args.file)[0]
        + ".csv. "
        + ("Included" if args.file_refs else "Excluded")
        + " odr and 3D model references."
    )
