"""Format output to be printed to a console."""

from enum import IntEnum, auto
from typing import List, Union


class Color:  # pylint: disable=too-few-public-methods
    """color codes for pretty printing"""

    PURPLE = "\033[95m"
    CYAN = "\033[96m"
    DARKCYAN = "\033[36m"
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    END = "\033[0m"


class _Status(IntEnum):
    """Enum for status logging, used by ``StatusFormatter``"""

    SUCCESS = auto()
    WARNING = auto()
    ERROR = auto()


def _format_color(
    color: Union[str, List[str]], str_: Union[str, List[str]]
) -> Union[str, List[str]]:
    """Format string to have color and or any other ANSI formatting.

    Args:
        color: Color (ANSI format) string
        str_: String to format
    Returns:
        Formatted string
    """
    if not isinstance(color, list):
        color = [color]

    if isinstance(str_, list):
        return ["".join(color) + s + Color.END for s in str_]
    return "".join(color) + str_ + Color.END


def format_green(str_: Union[str, List[str]]) -> Union[str, List[str]]:
    """Format ``str_`` to green string.

    Args:
        str_: String to format
    Returns:
        Formatted string
    """
    return _format_color(Color.GREEN, str_)


def format_yellow(str_: Union[str, List[str]]) -> Union[str, List[str]]:
    """Format ``str_`` to yellow string.

    Args:
        str_: String to format
    Returns:
        Formatted string
    """
    return _format_color(Color.YELLOW, str_)


def format_red(str_: Union[str, List[str]]) -> Union[str, List[str]]:
    """Format ``str_`` to red string.

    Args:
        str_: String to format
    Returns:
        Formatted string
    """
    return _format_color(Color.RED, str_)


def format_bold(str_: Union[str, List[str]]) -> Union[str, List[str]]:
    """Format ``str_`` to bold string.

    Args:
        str_: String to format
    Returns:
        Formatted string
    """
    return _format_color(Color.BOLD, str_)


def format_underline(str_: Union[str, List[str]]) -> Union[str, List[str]]:
    """Format ``str_`` to underlined string.

    Args:
        str_: String to format
    Returns:
        Formatted string
    """
    return _format_color(Color.UNDERLINE, str_)


def format_align(
    str_list: List[str], min_indentation: int, min_space: int
) -> List[str]:
    """Format list of strings to have equal length by adding spaces as padding.

    Args:
        str_list: List of strings
        min_indentation: Minimum length of string after formatting
        min_space: Minimum space to add to each string

    Returns:
        Formatted list of strings
    """
    max_str_len = max(len(str_) for str_ in str_list)

    if min_indentation > max_str_len + min_space:
        indentation = min_indentation
    else:
        indentation = max_str_len + min_space

    for i, str_ in enumerate(str_list):
        str_list[i] = str_list[i] + " " * (indentation - len(str_list[i]))

    return str_list
