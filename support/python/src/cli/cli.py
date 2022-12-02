"""Module for creating a CLI parser

This module is adapted for the needs of the ESMINI project.

It provides a wrapper around the argparse library, and enables extended features, such as:
    - ANSI style terminal output, such as colors
    - prettier printing of sub-command choices
    - listing of options
"""

# pylint: disable=too-many-lines
from __future__ import annotations
import argparse
import subprocess
import sys
from pathlib import Path
from typing import List

from support.python.src import formatter

VALID_ENTRYPOINTS = ["main.py"]


class _NoArgumentError(TypeError):
    def __init__(self, command_name):
        message_list = [
            "No Argument supplied for command '",
            formatter.format_bold(command_name),
            "'! Supply command(s) and/or argument(s).",
        ]
        formatted_message_str = formatter.format_red(message_list)
        super().__init__(formatted_message_str)


class _CommandValueError(ValueError):
    def __init__(self, command_name, message):
        message_list = formatter.format_red(
            [
                "Invalid command option '",
                formatter.format_bold(message),
                "' for command '",
                formatter.format_bold(command_name),
                "'. Choose a valid command option.",
            ]
        )

        formatted_message_str = formatter.format_red(message_list)
        super().__init__(formatted_message_str)


class _Arg:
    # pylint: disable=too-many-instance-attributes
    def __init__(
        self,
        name,
        help=None,  # pylint: disable=redefined-builtin
        action=None,
        dest=None,
        default=None,
        nargs=None,
        choices=None,
        metavar=None,
    ):
        # pylint: disable=too-many-arguments
        self._name = name
        self._help = help
        self._dest = dest
        self._action = action
        self._default = default
        self._nargs = nargs
        self._choices = choices
        self._metavar = metavar

    @property
    def name(self) -> str:
        """Get arg name

        Returns:
            name
        """
        return self._name

    @property
    def help(self) -> str:
        """Get arg help (description)

        Returns:
            help
        """
        return self._help

    @property
    def dest(self) -> str:
        """Get variable name to store arg as in dict, instead of arg ``name``

        Returns:
            dest
        """
        return self._dest

    @property
    def default(self) -> any:
        """Get default value for arg

        Returns:
            default
        """
        return self._default

    @property
    def nargs(self) -> str:
        """Get string descripbing What type of arg to store

        Returns:
            nargs
        """
        return self._nargs

    @property
    def choices(self) -> List[str]:
        """Get choices for arg

        Returns:
            choices
        """
        return self._choices

    @property
    def metavar(self) -> str:
        """Get display name for arg in help section

        Returns:
            metavar
        """
        return self._metavar

    @property
    def has_choices(self) -> bool:
        """Check if arg contains any choices

        Returns:
            has_choices
        """
        return self.choices is not None


class _Example:
    # pylint: disable=too-few-public-methods
    def __init__(self, arg_list):
        self._arg_list = arg_list

    @property
    def arg_list(self) -> List[str]:
        """get list of args to display in example command

        Returns:
            arg list
        """
        return self._arg_list


class _Command:
    # pylint: disable=too-many-instance-attributes
    PYTHON_VERSION = "python3.8"

    def __init__(
        self, name: str, help: str = None  # pylint: disable=redefined-builtin
    ) -> None:
        self._name = name
        self._help = help
        self._arg_list = []
        self._example_list = []
        self._parent = None
        self._child_list = []
        self._parser = argparse.ArgumentParser()
        self._parsed_args = None

    # ====================================
    #   Property methods
    # ====================================

    @property
    def name(self) -> str:
        """Get command name

        Returns:
            name
        """
        return self._name

    @property
    def help(self) -> str:
        """get command help (description)

        Returns:
            help
        """
        return self._help

    @property
    def arg_list(self) -> List[_Arg]:
        """get list of args

        Returns:
            arg_list
        """
        return self._arg_list

    @property
    def example_list(self) -> List[_Example]:
        """Get list of examples

        Returns:
            example_list
        """
        return self._example_list

    @property
    def parent(self) -> _Command:
        """Get Parent Command

        Returns:
            parent
        """
        return self._parent

    @parent.setter
    def parent(self, parent: _Command) -> None:
        """Set parent command

        Args:
            parent: parent command
        """
        self._parent = parent

    @property
    def child_list(self) -> List[_Command]:
        """Get list of child commands

        Returns:
            child_list
        """
        return self._child_list

    @property
    def parser(self) -> argparse.ArgumentParser:
        """Get parser.

        Parser is an ArgumentParser from the argparse library. ArgumentParser takes
         care of most of the parsing logic in this module.

        Returns:
            parser
        """
        return self._parser

    @property
    def parsed_args(self) -> dict:
        """Get parsed arguments

        Returns:
            parsed_args
        """
        return self._parsed_args

    # ====================================
    #   Derived properties
    # ====================================

    @property
    def ancestor_list(self) -> List[_Command]:
        """Get list of ancestor commands

        Returns:
            ancestor_list
        """
        ancestor_list = []
        command = self
        has_parent = True
        while has_parent:
            if command.parent is not None:
                # prepend ancestor
                ancestor_list.insert(0, command.parent)
                command = command.parent
            else:
                has_parent = False
        return ancestor_list

    @property
    def ancestor_name_list(self) -> List[str]:
        """Get list of ancestor command names

        Returns:
            ancestor_name_list
        """
        return [ancestor.name for ancestor in self.ancestor_list]

    @property
    def has_arg(self) -> bool:
        """Check if command has any arguments

        Returns:
            has_args
        """
        return len(self._arg_list) > 0

    @property
    def has_example(self) -> bool:
        """check if command has any examples

        Returns:
            has_example
        """
        return len(self._example_list) > 0

    @property
    def has_child(self) -> bool:
        """Check if command has any child commands

        Returns:
            has_child
        """
        return len(self._child_list) > 0

    @property
    def has_any_arg_choices(self) -> bool:
        """Check is any command argument has any choices

        Returns:
            has_any_arg_choices
        """
        if self.has_arg:
            for arg in self.arg_list:
                if arg.has_choices:
                    return True
        return False

    # ====================================
    #   Command config methods
    # ====================================

    def add_child(
        self, name: str, help: str = None  # pylint: disable=redefined-builtin
    ) -> _Command:
        """Add child command

        Args:
            name: command name
            help: command help (description)

        Returns:
            child command
        """
        child = _Command(name, help=help)
        child.parent = self
        self._child_list.append(child)
        return child

    def add_argument(self, *args: any, **kwargs: any) -> None:
        """Add argument to command

        Args:
            *args: arguments
            **kwargs: keyword arguments

        *args:
        ------
            name_or_flags:
                Either a name or a list of option strings, e.g. foo or -f, --foo.

        **kwargs:
        ------
        action:
            The basic type of action to be taken when this argument is encountered at the command
            line.
        nargs:
            The number of command-line arguments that should be consumed.
        const:
            A constant value required by some action and nargs selections.
        default:
            The value produced if the argument is absent from the command line.
        type:
            The type to which the command-line argument should be converted.
        choices:
            A container of the allowable values for the argument.
        required:
            Whether or not the command-line option may be omitted (optionals only).
        help:
            A brief description of what the argument does.
        metavar:
            A name for the argument in usage messages.
        dest:
            The name of the attribute to be added to the object returned by parse_args().

        """
        self._arg_list.append(_Arg(*args, **kwargs))
        self._parser.add_argument(*args, **kwargs)

    def add_example(self, arg_list: List[str]) -> None:
        """Add examples of arg combinations to command

        Args:
            arg_list: list of strings to build examples from
        """
        self._example_list.append(_Example(arg_list))

    # ====================================
    #   Parsing methods
    # ====================================

    def parse_arg_list(self, argv: List[str]) -> _Command:
        """Parse arguments

        Args:
            argv: list of arguments

        Returns:
            dict of parsed arguments
        """
        # Initialize argument parser
        self._setup_parser()
        parsed_command = self
        # Parse command and child command
        if self.has_child:

            # get parsed args
            if len(argv) < 1:
                self._parser.print_help()
                raise _NoArgumentError(self.name)
            args = self._parser.parse_args([argv[0]])

            # Raise error if argument is not a valid child command
            if args.command not in [child.name for child in self.child_list]:
                self._parser.print_help()
                raise _CommandValueError(self.name, args.command)

            # Recursively continue parsing child command whose name matches arg.command
            for child in self.child_list:
                if child.name == args.command:
                    parsed_command = child.parse_arg_list(argv[1:])

        else:
            # Stop recursion, parse and return command
            if "--list" in argv or "-l" in argv:
                self._print_argument_options()
                sys.exit(0)

            self._parsed_args = self._parser.parse_args(argv).__dict__
            if "list" in self._parsed_args.keys():
                self._parsed_args.pop("list")

        return parsed_command

    # ===========================================
    #   Private parsing methods
    # ===========================================

    def _setup_parser(self):

        # Setup argument parser
        self._parser.usage = self._get_usage_str()
        self._parser.formatter_class = argparse.RawTextHelpFormatter

        # Add parser command and info
        self._add_parser_command()

        # Add --list argument
        if self.has_any_arg_choices:
            self._parser.add_argument(
                "-l",
                "--list",
                action="store_true",
                dest="list",
                help="list argument options",
            )

    def _add_parser_command(self):
        if self.has_child:
            self._parser.add_argument(
                metavar="<command>",
                help="select an available option from "
                + formatter.format_bold("<command> options"),
                dest="command",
            )

    def _print_argument_options(self):
        print(self._get_argument_options_str())

    # ===========================================
    #   Usage string formatting
    # ===========================================

    def _get_usage_str(self):
        str_ = ""
        str_ += self._get_command_info_str()
        if self.has_child:
            str_ += self._get_entry_options_str()
            str_ += self._get_command_options_str()
        str_ += self._add_example_run_info_str()
        return str_

    def _get_command_info_str(self):
        ancestor_name_list = formatter.format_green(self.ancestor_name_list)
        name = formatter.format_green(self.name)
        if self.has_child:
            command_literal = formatter.format_green("<command>")
        else:
            command_literal = None
        args_literal = "[<args>]"

        # Create a list of the components so that we can join them and provide space " " in between
        str_list = [
            self.PYTHON_VERSION,
            " ".join(ancestor_name_list),
            name,
            command_literal,
            args_literal,
        ]

        # Remove empty strings from list, so that we don't get extra spaces " " in between words
        str_list_filtered = list(filter(None, str_list))

        # Join substring to one string and add a new line character
        str_ = " ".join(str_list_filtered) + "\n"
        str_ += "\n"
        str_ += self.help + "\n"
        str_ += "\n"
        return str_

    def _get_command_options_str(self):
        command_name_list = ["  " + child.name for child in self.child_list]
        command_help_list = [child.help for child in self.child_list]

        command_name_list = formatter.format_align(
            command_name_list,
            22,
            5,
        )

        str_ = formatter.format_bold("<command> options:\n")
        for name, help in zip(  # pylint: disable=redefined-builtin
            command_name_list, command_help_list
        ):
            if (
                name.strip() != "--env"
                and name.strip() != "--nointeractive"
                and name.strip() != "--version"
                and name.strip() != "--mount_ccache"
            ):
                str_ += formatter.format_green(name) + help + "\n"

        str_ += "\n"
        return str_

    def _get_entry_options_str(self):
        command_name_list = ["  " + child.name for child in self.child_list]
        command_help_list = [child.help for child in self.child_list]

        command_name_list = formatter.format_align(
            command_name_list,
            22,
            5,
        )

        str_ = formatter.format_bold("optional <entry> options:\n")
        for name, help in zip(  # pylint: disable=redefined-builtin
            command_name_list, command_help_list
        ):
            if (
                name.strip() == "--env"
                or name.strip() == "--nointeractive"
                or name.strip() == "--version"
                or name.strip() == "--mount_ccache"
            ):
                str_ += formatter.format_green(name) + help + "\n"
        str_ += "\n"
        return str_

    def _add_example_run_info_str(self):
        str_ = ""

        example_command_str_list = []
        example_arg_str_list = []

        self._get_command_example_str_list(
            example_command_str_list, example_arg_str_list
        )

        if self.has_child:
            self._get_child_command_example_str_list(
                example_command_str_list, example_arg_str_list
            )

        if example_command_str_list:
            example_command_str_list = formatter.format_align(
                example_command_str_list,
                22,
                5,
            )

            complete_example_str_list = []
            for cmd, arg in zip(example_command_str_list, example_arg_str_list):
                if (
                    "--env" not in cmd
                    and "--nointeractive" not in cmd
                    and "--version" not in cmd
                    and "--mount_ccache" not in cmd
                ):
                    complete_example_str_list.append("".join([cmd, arg]))

            # Join substring to one string and add a new line character
            str_ += "\n".join(["  " + str_ for str_ in complete_example_str_list])
        if str_:
            str_ = formatter.format_bold("Example runs:\n") + str_
            str_ += "\n"
        return str_

    def _get_command_example_str_list(
        self, example_command_str_list, example_arg_str_list
    ):
        ancestor_name_list = formatter.format_green(self.ancestor_name_list)
        command_name = formatter.format_green(self.name)

        if self.has_example:
            for example in self.example_list:
                # Create a list of the components so that we can join them and provide space " "
                # in between Remove empty strings from list, so that we don't get extra spaces "
                # " in between words
                example_command_str = " ".join(
                    list(
                        filter(
                            None,
                            [
                                self.PYTHON_VERSION,
                                " ".join(ancestor_name_list),
                                command_name,
                            ],
                        )
                    )
                )
                example_command_str_list.append(example_command_str)
                example_arg_str_list.append(" ".join(example.arg_list))

    def _get_child_command_example_str_list(
        self, example_command_str_list, example_arg_str_list
    ):
        ancestor_name_list = formatter.format_green(self.ancestor_name_list)
        command_name = formatter.format_green(self.name)

        for child in self.child_list:
            # if child.has_example:
            child_name = formatter.format_green(child.name)
            # N.B. only shows the first sub command example in the command help example =
            # child.example_list[0] example_arg_list = _Formatter.example_arg_list(
            # example.arg_list) Create a list of the components so that we can join them and
            # provide space " " in between Remove empty strings from list, so that we don't get
            # extra spaces " " in between words
            example_command_str = " ".join(
                list(
                    filter(
                        None,
                        [
                            self.PYTHON_VERSION,
                            " ".join(ancestor_name_list),
                            command_name,
                            child_name,
                        ],
                    )
                )
            )
            example_command_str_list.append(example_command_str)

            example_arg_str_list.append("[<args>]")

    def _get_argument_options_str(self):
        str_ = ""
        for arg in self.arg_list:
            if arg.has_choices:
                str_ += formatter.format_bold(
                    "Available options for " + arg.name + ":\n"
                )
                for choice in arg.choices:
                    str_ += "  " + choice + "\n"
        return str_


class CLI:
    """Build CLI from commands and parse CLI arguments"""

    def __init__(self):
        self._root_command = _Command(
            "main.py",
            "ESMINI command line entry point",
        )

        self._init_setup()
        self._init_fetch()
        self._init_replace()
        self._init_generate()
        self._init_run()

    def _init_setup(self):
        setup_cmd = self._root_command.add_child(
            "setup", "setups the project from scratch"
        )

        setup_cmd.add_example([])
        setup_cmd.add_example(["--clean"])

        setup_cmd.add_argument(
            "--clean",
            action="store_true",
            help="setups from scratch in clean mode",
        )

    def _init_fetch(self):
        fetch_cmd = self._root_command.add_child("fetch", "fetches desired entities")

        # dependency
        fetch_dependency_cmd = fetch_cmd.add_child(
            "dependency", "fetches desired dependencies"
        )
        fetch_dependency_cmd.add_example(["--name osi"])
        fetch_dependency_cmd.add_example(["--name osi", "--source google-drive"])

        fetch_dependency_cmd.add_argument(
            "--name",
            dest="name",
            default="all",
            nargs="?",
            help="select the name of the dependency (default: %(default)s)",
            choices=[
                "osg",
                "osi",
                "sumo",
                "googletest",
                "models",
            ],
            metavar="NAME",
        )

        fetch_dependency_cmd.add_argument(
            "--source",
            dest="source",
            default="google-drive",
            nargs="?",
            help="select the source for dependency (default: %(default)s)",
            choices=[
                "google-drive",
                "dropbox",
                "esmini",
            ],
            metavar="SOURCE",
        )

    def _init_replace(self):
        replace_cmd = self._root_command.add_child(
            "replace", "replaces desired entities"
        )

        # dependency
        replace_dependency_cmd = replace_cmd.add_child(
            "dependency", "replaces desired dependencies"
        )
        replace_dependency_cmd.add_example(["--name osi"])
        replace_dependency_cmd.add_example(["--name osi", "--source google-drive"])

        replace_dependency_cmd.add_argument(
            "--name",
            dest="name",
            default="all",
            nargs="?",
            help="select the name of the dependency (default: %(default)s)",
            choices=[
                "osg",
                "osi",
                "sumo",
                "googletest",
                "models",
            ],
            metavar="NAME",
        )

        replace_dependency_cmd.add_argument(
            "--source",
            dest="source",
            default="google-drive",
            nargs="?",
            help="Select the source for dependency (default: %(default)s)",
            choices=[
                "google-drive",
                "dropbox",
                "esmini",
            ],
            metavar="SOURCE",
        )

    def _init_generate(self):
        generate_cmd = self._root_command.add_child(
            "generate", "generates desired entities"
        )

        # openscenario
        generate_openscenario_cmd = generate_cmd.add_child(
            "openscenario", "generates the C++ classes to reflect OpenScenario standard"
        )
        generate_openscenario_cmd.add_example([""])
        generate_openscenario_cmd.add_example(["--1.2"])
        generate_openscenario_cmd.add_example(["--1.1"])

        generate_openscenario_cmd.add_argument(
            "--1.2",
            action="store_true",
            dest="1.2",
            help="generates the C++ classes based on OpenScenario 1.2",
        )
        generate_openscenario_cmd.add_argument(
            "--1.1",
            action="store_true",
            dest="1.1",
            help="generates the C++ classes based on OpenScenario 1.1",
        )

        # opendrive
        generate_opendrive_cmd = generate_cmd.add_child(
            "opendrive", "generates the C++ classes to reflect OpenDrive standard"
        )
        generate_opendrive_cmd.add_example([""])
        generate_opendrive_cmd.add_example(["--1.7"])
        generate_opendrive_cmd.add_example(["--1.6"])
        generate_opendrive_cmd.add_example(["--1.5"])

        generate_opendrive_cmd.add_argument(
            "--1.7",
            action="store_true",
            dest="1.7",
            help="generates the C++ classes based on OpenDrive 1.7",
        )
        generate_opendrive_cmd.add_argument(
            "--1.6",
            action="store_true",
            dest="1.6",
            help="generates the C++ classes based on OpenDrive 1.6",
        )
        generate_opendrive_cmd.add_argument(
            "--1.5",
            action="store_true",
            dest="1.5",
            help="generates the C++ classes based on OpenDrive 1.5",
        )

    def _init_run(self):
        run_cmd = self._root_command.add_child("run", "runs with desired options")

        # format
        run_format_cmd = run_cmd.add_child("format", "formats code")
        run_format_cmd.add_example([])
        run_format_cmd.add_example(["--clang_format"])
        run_format_cmd.add_example(["--clang_format_checker"])
        run_format_cmd.add_example(["--cmake_format"])
        run_format_cmd.add_example(["--cmake_format_checker"])
        run_format_cmd.add_example(["--black_format"])
        run_format_cmd.add_example(["--black_format_checker"])

        run_format_cmd.add_argument(
            "--clang_format",
            action="store_true",
            help="runs clang-format, apply changes (.cpp/.c/.hpp/.h)",
        )

        run_format_cmd.add_argument(
            "--clang_format_checker",
            action="store_true",
            help="runs clang-format, does not apply changes, only make checks (.cpp/.c/.hpp/.h)",
        )

        run_format_cmd.add_argument(
            "--cmake_format",
            action="store_true",
            help="runs cmake-format, apply changes (CMakeLists.txt)",
        )

        run_format_cmd.add_argument(
            "--cmake_format_checker",
            action="store_true",
            help="runs cmake-format, does not apply changes, only make checks (CMakeLists.txt)",
        )

        run_format_cmd.add_argument(
            "--black_format",
            action="store_true",
            help="runs black, apply changes (.py)",
        )

        run_format_cmd.add_argument(
            "--black_format_checker",
            action="store_true",
            help="runs black, does not apply changes, only make checks (.py)",
        )

        # Pylint
        run_lint_cmd = run_cmd.add_child("pylint", "lint checks python source code")
        run_lint_cmd.add_example([""])

        # Pytest
        run_pytest_cmd = run_cmd.add_child("pytest", "tests python source code")
        run_pytest_cmd.add_example([""])

    def parse(self, argv: List[str]) -> List[_Command]:
        """Parse CLI arguments

        Args:
            argv: list of CLI arguments

        Returns:
            List of all called sub-commands, containing parsed CLI arguments
        """

        if Path(argv[0]).name in VALID_ENTRYPOINTS:
            leaf_command = self._root_command.parse_arg_list(argv[1:])
            command_list = leaf_command.ancestor_list
            command_list.append(leaf_command)
            return command_list

        raise ValueError(
            "'"
            + argv[0]
            + "'"
            + " is not a valid command line entry point. Expected 'main.py'"
        )
