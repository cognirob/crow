import curses
import itertools
import os
import locale
locale.setlocale(locale.LC_ALL, '')

import rclpy
from crow_msgs.msg import SentenceProgram
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

if "MWX_DEBUG" in os.environ:
    import pydevd_pycharm

    pydevd_pycharm.settrace('localhost', port=25565, stdoutToServer=True, stderrToServer=True)


code = locale.getpreferredencoding()

class BINDINGS:
    bindings = {
        "command": {'1': 'ukaž', '2': 'seber', '3': 'polož', '4': 'podej', '5': 'ukliď',
                    '6': 'pustit', '7': 'úložiště', '8': 'pozice', '9': 'Odstraň poslední příkaz',
                    '0': 'Odstraň příkaz', ' ': ' '},
        "object": {'q': 'kostka', 'w': 'kolo', 'e': 'střecha', 'r': 'lžíce', 't': 'koule', 'y': 'destička',
                   'u': 'matka', 'i': 'šroub', 'm': 'klíč', 'n': 'šroubovák', 'b': 'kleště', 'v': 'kladivo',
                   'c': 'to', ' ': ' '},
        "color": {'a': 'červená', 's': 'vínová', 'd': 'modrá', 'f': 'zelená', 'g': 'fialová', 'h': 'zlatá',
                  'j': 'bílá', ' ': ' '},
        "location": {'z': 'sklad', 'x': 'stůl', ' ': ' '},
    }

    bindings_arr = [bindings["command"], bindings["object"], bindings["color"], bindings["location"]]


class DummyNlInput(Node):

    def __init__(self, node_name="dummy_nl_input"):
        super().__init__(node_name)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.nl_publisher = self.create_publisher(SentenceProgram, "/nl_input", qos)

    def publish_command(self, command):
        msg = SentenceProgram()
        msg.data.append(command)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.nl_publisher.publish(msg)
        self.get_logger().info(f'Publishing text {msg.data}')


def render_table(win, location, header, table, column_sizes):
    x = location[0]
    y = location[1]

    win.clear()

    pos = 0
    for head, column_size in zip(header, column_sizes):
        win.addstr(y + 0, x + pos, head)
        pos += column_size

    for line, i in zip(table, range(len(table))):
        pos = 0
        for cell, column_size in zip(line, column_sizes):
            win.addstr(y + i + 1, x + pos, str(cell))
            pos += column_size

    win.refresh()


def render_legend(legend):
    legend_table = []
    legend.clear()

    for com, obj, col, loc in itertools.zip_longest(*map(lambda x: x.keys(), BINDINGS.bindings_arr)):

        legend_table.append([])
        for key, binding_dict in zip([com, obj, col, loc], BINDINGS.bindings_arr):
            if key is None:
                legend_table[-1].append("")
                continue

            legend_table[-1].append(f"{key}:{binding_dict[key]}")

    render_table(legend, (0, 0),
                 header=["command", "object", "color", "location"],
                 table=legend_table,
                 column_sizes=[40, 20, 20, 20])

    legend.refresh()


def render_usage(usage, mode):
    usage.clear()
    usage.addstr(0, 0, "Hello! Use the following table to navigate. Use <F4> to exit or use CTRL^C. ")
    usage.addstr(1, 0, "Use <F5> to switch between quick and normal mode")
    usage.addstr(2, 0, f"Mode: '{mode}'")
    usage.refresh()


def translate_query(chars):
    query = ""
    for binding_dict in BINDINGS.bindings_arr:
        for c in chars:
            if c in binding_dict:
                query += binding_dict[c] + " "
    query = query.strip()
    return query


def render_command(win_command, win_query, chars, query):
    win_query.clear()
    win_query.addstr(0, 0, f"{query}")
    win_query.refresh()

    win_command.clear()
    win_command.addstr(0, 0, f":{chars}")
    win_command.refresh()


def render_history(history_window, history):
    history_window.clear()
    rows, cols = history_window.getmaxyx()
    printable = history[-rows:]

    for i, c in enumerate(printable):
        history_window.addstr(i, 0, f"{i}: {c[2]}" + (f" ({c[0]})" if c[1] == 'quick' else ''))

    history_window.refresh()


def app(stdscr):
    rclpy.init()
    node = DummyNlInput()

    # PREPARE WINDOW
    rows, cols = stdscr.getmaxyx()
    sizes = [4, 20, 10, 1, 1]
    wins = []
    history = []
    history_pos = 0

    row = 0
    for i in sizes:
        wins.append(curses.newwin(i, cols, row, 0))
        row += i

    win_usage, win_legend, win_logs, win_command, win_query = wins

    mode = "quick"
    chars = ""
    query = ""

    while True:
        render_usage(win_usage, mode)
        render_legend(win_legend)
        render_history(win_logs, history)
        render_command(win_command, win_query, chars, query)

        ch = stdscr.get_wch()
        if type(ch) is int:
            ch = curses.keyname(ch).decode()

        if ch == 'KEY_F(5)':
            mode = 'normal' if mode == 'quick' else 'quick'
            chars = ""
            history_pos = 0
        elif ch == 'KEY_F(4)':
            break
        elif ch == 'KEY_BACKSPACE':
            chars = chars[:-1]
            history_pos = 0
        elif ch == 'KEY_UP':
            history_pos += 1
            history_pos = min(history_pos, len(history))
        elif ch == 'KEY_DOWN':
            history_pos -= 1
            history_pos = max(0, history_pos)
        elif not ch == '\n':
            chars += ch
            history_pos = 0

        if history_pos != 0:
            chars, mode, query = history[-history_pos]

        if mode == 'quick':
            query = translate_query(chars)
        else:
            query = chars#.encode("utf-8")

        render_command(win_command, win_query, chars, query)

        if ch == '\n':
            node.publish_command(query)
            history.append((chars, mode, query))
            chars = ""
            query = ""
            history_pos = 0

    node.destroy_node()


def main():
    curses.wrapper(app)


if __name__ == "__main__":
    main()
