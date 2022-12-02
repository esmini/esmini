""" dependency.py handles the all the entities in the project that are classified as dependency

"""

from support.python.src.command.fetch.utils import (
    fetch_from_given_source,
    replace_from_given_source,
)
from support.python.src.globals import ESMINI_DEPENDENCY_EXTERNALS
from support.python.src.utils import print_commands, get_os


class Dependency:
    """dependency command 'namespace'"""

    @staticmethod
    def fetch_dependency(key, args):
        """fetches the requested dependency from desired source"""

        Dependency._resolve_dependency_args(args)
        if args["name"] == "all" or args["name"][0] == "all":
            for dep_name, dep_data in ESMINI_DEPENDENCY_EXTERNALS[args["source"]][
                get_os()
            ].items():
                fetch_from_given_source(key, args["source"], dep_name, dep_data)
        else:
            for desired_name in args["name"]:
                for dep_name, dep_data in ESMINI_DEPENDENCY_EXTERNALS[args["source"]][
                    get_os()
                ].items():
                    if dep_name == desired_name:
                        fetch_from_given_source(key, args["source"], dep_name, dep_data)

    @staticmethod
    def replace_dependency(args):
        """replaces the requested dependency with desired methods"""

        version = ""
        Dependency._resolve_dependency_args(args)
        if args["name"] == "all" or args["name"][0] == "all":
            for dep_name, dep_data in ESMINI_DEPENDENCY_EXTERNALS[args["source"]][
                get_os()
            ].items():
                replace_from_given_source(args["source"], dep_name, dep_data, version)
        else:
            for desired_name in args["name"]:
                for dep_name, dep_data in ESMINI_DEPENDENCY_EXTERNALS[args["source"]][
                    get_os()
                ].items():
                    if dep_name == desired_name:
                        replace_from_given_source(
                            args["source"], dep_name, dep_data, version
                        )

    @staticmethod
    def _resolve_dependency_args(args):
        """resolves the dependency arguments in case there is conflicted request"""

        if args["name"] != "all" and args["name"][0] != "all":
            match = False
            if isinstance(args["name"], str):
                args["name"] = [args["name"]]

            for desired_dep in args["name"]:
                for dep in ESMINI_DEPENDENCY_EXTERNALS[args["source"]][get_os()].keys():
                    if dep == desired_dep:
                        match = dep
                if not match:
                    raise ValueError(
                        "DEPENDENCY NOT AVAILABLE",
                        "SEE HELP SECTION VIA",
                        "python3.8 start.py fetch dependency --help",
                    )

        print_commands(args)
