last_line_length = 0


def print_updateable_line(data):
    global last_line_length
    print(" " * last_line_length, end="\r")
    print(data, end="\r")
    last_line_length = len(data)
