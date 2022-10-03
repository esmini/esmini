from dat import *

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Read and print .dat file')

    # Add the arguments
    parser.add_argument('filename', help='dat filename')
    parser.add_argument('--extended', '-e', action='store_true', help='add road coordinates')

    # Execute the parse_args() method
    args = parser.parse_args()

    dat = DATFile(args.filename)
    dat.save_csv(extended = True if args.extended else False)
    print('Created ' + os.path.splitext(args.filename)[0] + '.csv')
    dat.close()

