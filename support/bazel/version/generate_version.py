import argparse
import sys
import re


def generate_version(in_args):

    with open(in_args["version_file"], "r", encoding="utf-8") as version_file:
        version_data = version_file.read()

    git_commit = re.findall(r"GIT_COMMIT.*", version_data, re.DOTALL)
    commit_id = git_commit[0].split(" ")[1]
    print(commit_id)

    git_tag = re.findall(r"GIT_TAG.*", version_data, re.DOTALL)
    tag = git_tag[0].split(" ")[1].split("\n")[0]
    print(tag)

    version_cpp = f'const char* ESMINI_GIT_REV       = "{tag}-{commit_id}"; \n\
const char* ESMINI_GIT_TAG       = "{tag}"; \n\
const char* ESMINI_GIT_BRANCH    = "tags/{tag}"; \n\
const char* ESMINI_BUILD_VERSION = "N/A - client build";'

    with open(in_args["gen_version"], "w", encoding="utf-8") as gen_version:
        gen_version.write(version_cpp)

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("version_file")
    parser.add_argument("gen_version")

    args = parser.parse_args(argv)
    generate_version(args.__dict__)


if __name__ == "__main__":
    main(sys.argv[1:])
    sys.exit(0)

