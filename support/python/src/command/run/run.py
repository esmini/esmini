# pylint: disable=duplicate-code
""" run.py handles the run operations within this repo """  # pylint: disable=too-many-lines

import glob
import os
from support.python.src import formatter
from support.python.src.utils import print_commands, subprocess_popen
from support.python.src.globals import (
    SEPARATOR,
    ESMINI_CLANG_FORMAT_EXCLUDES,
    ESMINI_CLANG_FORMAT_EXTENSION,
    ESMINI_CLANG_FORMAT_INCLUDES,
    ESMINI_CMAKE_FORMAT_INCLUDES,
    ESMINI_CMAKE_FORMAT_EXTENSION,
    ESMINI_DIRECTORY_ROOT,
)

############################################################################################################################## # pylint: disable=line-too-long
####################################################### Run CLASS ############################################################ # pylint: disable=line-too-long
############################################################################################################################## # pylint: disable=line-too-long


class Run:  # pylint: disable=too-many-instance-attributes, too-many-public-methods
    """Run command context"""

    def __init__(self):
        """initializes run class to perform running operations

        Parameters
        ----------
            format_args (dict): format arguments in dictionary format

        """

        self.format_args = {}

    def resolve_format_args(self):
        """resolves the format arguments in case there is conflicted request"""

        if (  # pylint: disable=too-many-boolean-expressions
            not self.format_args["clang_format"]
            and not self.format_args["black_format"]
            and not self.format_args["cmake_format"]
            and not self.format_args["clang_format_checker"]
            and not self.format_args["black_format_checker"]
            and not self.format_args["cmake_format_checker"]
        ):
            self.format_args["clang_format"] = True
            self.format_args["black_format"] = True
            self.format_args["cmake_format"] = True

        if (
            self.format_args["clang_format"]
            and self.format_args["clang_format_checker"]
        ):
            raise ValueError(
                formatter.format_red(
                    "You must select either clang_format or clang_format_checker"
                )
            )

        if (
            self.format_args["black_format"]
            and self.format_args["black_format_checker"]
        ):
            raise ValueError(
                formatter.format_red(
                    "You must select either black_format or black_format_checker"
                )
            )

        if (
            self.format_args["cmake_format"]
            and self.format_args["cmake_format_checker"]
        ):
            raise ValueError(
                formatter.format_red(
                    "You must select either cmake_format or cmake_format_checker"
                )
            )

        print_commands(self.format_args)

    @staticmethod
    def get_files_for_clang_format(includes, excludes):
        """get_files_for_clang_format sets all desired files to be formatted by clang-format

        Inputs
        ----------
            includes (list): include folder list

            excludes (list): exclude folder list

        Return
        ------
            all_trimmed_files (list): all files that are generated based on the given request

        """

        all_files = []
        for inc in includes:
            for ext in ESMINI_CLANG_FORMAT_EXTENSION:
                collect = glob.glob(os.path.join(inc, "**", "*" + ext), recursive=True)
                all_files = all_files + collect

        all_trimmed_files = []
        for file in all_files:
            exist = False
            for exc in excludes:
                if exc in file:
                    exist = True

            if not exist:
                all_trimmed_files.append(file)

        print("─" * SEPARATOR)
        return all_trimmed_files

    @staticmethod
    def get_files_for_cmake_format():
        """get_files_for_cmake_format sets all desired files to be formatted by cmake-format

        Return
        ------
            all_files (list): all files that are generated based on the given request

        """

        all_files = []
        for inc in ESMINI_CMAKE_FORMAT_INCLUDES:
            if os.path.isfile(inc):
                all_files.append(inc)
            elif os.path.isdir(inc):
                for ext in ESMINI_CMAKE_FORMAT_EXTENSION:
                    collect = glob.glob(
                        os.path.join(inc, "**", "*" + ext), recursive=True
                    )
                    all_files = all_files + collect

        return all_files

    @staticmethod
    def check_dot_clang_format_file():
        """check_dot_clang_format_file checks if there is a configuration file called .clang-format"""  # pylint: disable=line-too-long

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)
        if os.path.exists(os.path.join(ESMINI_DIRECTORY_ROOT, ".clang-format")):
            print(
                formatter.format_green(
                    os.path.join(ESMINI_DIRECTORY_ROOT, ".clang-format")
                    + " is selected as clang-format configuration",
                )
            )
            os.chdir(root_dir)
        else:
            raise FileExistsError(
                formatter.format_red(
                    "CONFIGURATION FILE "
                    + ".clang-format "
                    + "DOES NOT EXIST UNDER "
                    + ESMINI_DIRECTORY_ROOT
                )
            )

    @staticmethod
    def check_dot_cmake_format_file():
        """check_dot_cmake_format_file checks if there is a configuration file called .cmake-format"""  # pylint: disable=line-too-long

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)
        if os.path.exists(os.path.join(ESMINI_DIRECTORY_ROOT, ".cmake-format")):
            print(
                formatter.format_green(
                    os.path.join(ESMINI_DIRECTORY_ROOT, ".cmake-format")
                    + " is selected as cmake-format configuration",
                )
            )
            os.chdir(root_dir)
        else:
            raise FileExistsError(
                formatter.format_red(
                    "CONFIGURATION FILE "
                    + ".cmake-format "
                    + "DOES NOT EXIST UNDER "
                    + ESMINI_DIRECTORY_ROOT
                )
            )

    @staticmethod
    def check_dot_black_format_file():
        """check_dot_black_format_file checks if there is a configuration file called .black-format"""  # pylint: disable=line-too-long

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)
        if os.path.exists(os.path.join(ESMINI_DIRECTORY_ROOT, ".black-format")):
            print(
                formatter.format_green(
                    os.path.join(ESMINI_DIRECTORY_ROOT, ".black-format")
                    + "is selected as black-format configuration",
                )
            )
            print("─" * SEPARATOR)
            os.chdir(root_dir)
        else:
            raise FileExistsError(
                formatter.format_red(
                    "CONFIGURATION FILE "
                    + ".black-format "
                    + "DOES NOT EXIST UNDER "
                    + ESMINI_DIRECTORY_ROOT,
                )
            )

    @staticmethod
    def run_clang_format(includes, excludes, silent_mode):
        """run_clang_format runs the clang format

        Inputs
        ----------
            includes (list): include folder list

            excludes (list): exclude folder list

            silent_mode (bool): enables / disables printing
        """

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_clang_format_file()
        files = Run.get_files_for_clang_format(includes, excludes)

        for file in files:
            os.system("clang-format-15 -style=file -i " + file)
            if not silent_mode:
                print(
                    formatter.format_green("Formatted: ")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_green(os.path.basename(file))
                )
        print("─" * SEPARATOR)
        os.chdir(root_dir)

    @staticmethod
    def run_clang_format_checker():
        """run_clang_format runs the clang format"""

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_clang_format_file()
        files = Run.get_files_for_clang_format(
            ESMINI_CLANG_FORMAT_INCLUDES, ESMINI_CLANG_FORMAT_EXCLUDES
        )

        counter = 0
        for file in files:
            stdout, stderr = subprocess_popen(
                ["clang-format-15", "--dry-run", "-style=file", "-i", file]
            )

            if len(stdout) == 0 and len(stderr) == 0:
                print(
                    formatter.format_green("Already Formatted: ")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_green(os.path.basename(file))
                )
            else:
                counter = counter + 1
                print(
                    formatter.format_red("Requires Formatting: ")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_red(os.path.basename(file))
                )
        print("─" * SEPARATOR)
        if counter > 0:
            raise ValueError(
                "Format your code with clang-format. Unformatted files are detected. Check the log."
            )
        os.chdir(root_dir)

    @staticmethod
    def run_cmake_format():
        """run_cmake_format runs the cmake format"""

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_cmake_format_file()
        files = Run.get_files_for_cmake_format()

        for file in files:
            os.system(
                "cmake-format -c "
                + os.path.join(ESMINI_DIRECTORY_ROOT, ".cmake-format")
                + " -i "
                + file
            )
            print(
                formatter.format_green("Formatted: ")
                + os.path.dirname(file)
                + "/"
                + formatter.format_green(os.path.basename(file))
            )
        print("─" * SEPARATOR)
        os.chdir(root_dir)

    @staticmethod
    def run_cmake_format_checker():
        """run_cmake_format_checker runs the cmake format"""

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_cmake_format_file()
        files = Run.get_files_for_cmake_format()

        counter = 0
        for file in files:
            os.chdir(os.path.split(file)[0])
            _, stderr = subprocess_popen(
                [
                    "cmake-format",
                    "--check",
                    file,
                    "-c",
                    os.path.join(ESMINI_DIRECTORY_ROOT, ".cmake-format"),
                ],
            )

            if len(stderr) == 0:
                print(
                    formatter.format_green("Already Formatted: ")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_green(os.path.basename(file))
                )
            else:
                counter = counter + 1
                print(
                    formatter.format_red(
                        "Requires Formatting: "
                        + stderr.split("Check failed: ", maxsplit=1)[-1].split("\n")[0]
                    )
                )
        print("─" * SEPARATOR)
        if counter > 0:
            raise ValueError(
                "Format your code with cmake-format. Unformatted files are detected. Check the log."
            )
        os.chdir(root_dir)

    @staticmethod
    def run_black_format():
        """run_black_format runs the "black" and apply changes"""

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_black_format_file()
        is_black_done = os.system(
            "black . --config=" + os.path.join(ESMINI_DIRECTORY_ROOT, ".black-format")
        )
        if is_black_done == 0:
            print(formatter.format_green("BLACK FORMATING DONE"))
        else:
            raise ValueError(formatter.format_red("BLACK FORMATING FAILED"))
        print("─" * SEPARATOR)
        os.chdir(root_dir)

    @staticmethod
    def run_black_format_checker():
        """run_black_format_checker runs the "black" and does not apply changes"""

        root_dir = os.getcwd()
        os.chdir(ESMINI_DIRECTORY_ROOT)

        Run.check_dot_black_format_file()
        is_black_done = os.system(
            "black . --config="
            + os.path.join(ESMINI_DIRECTORY_ROOT, ".black-format")
            + " --check"
        )
        if is_black_done == 0:
            print(formatter.format_green("BLACK FORMATING CHECK", "DONE"))
        else:
            raise ValueError(formatter.format_red("BLACK FORMATING CHECK", "FAILED"))
        print("─" * SEPARATOR)
        os.chdir(root_dir)

    def run_format(self):
        """run_format runs the requested run format command"""

        self.resolve_format_args()
        if self.format_args["clang_format"]:
            Run.run_clang_format(
                ESMINI_CLANG_FORMAT_INCLUDES, ESMINI_CLANG_FORMAT_EXCLUDES, False
            )
        if self.format_args["clang_format_checker"]:
            Run.run_clang_format_checker()
        if self.format_args["cmake_format"]:
            Run.run_cmake_format()
        if self.format_args["cmake_format_checker"]:
            Run.run_cmake_format_checker()
        if self.format_args["black_format"]:
            Run.run_black_format()
        if self.format_args["black_format_checker"]:
            Run.run_black_format_checker()
