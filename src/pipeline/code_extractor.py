import re


def extract_code_from_txt(text):
    # This regex matches a code block between triple backticks.
    # It also optionally skips the language specifier after the opening backticks.
    pattern = r"```(?:\w*\n)?(.*?)```"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        return match.group(1).strip()
    return None