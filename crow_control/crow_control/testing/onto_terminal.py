from typing import List, Tuple
from textwrap import wrap
import npyscreen
from crow_ontology.crowracle_client import CrowtologyClient
import curses
import inspect
import re
from itertools import chain
import os


class History():
    MAX_LEN = 100
    HISTORY_FILE = ".oterm_history.txt"

    def __init__(self) -> None:
        if not os.path.isfile(self.HISTORY_FILE):
            with open(self.HISTORY_FILE, "w") as f:
                pass

        with open(self.HISTORY_FILE, "r+") as f:
            if f.readable():
                self.__hist_list = [h.strip() for h in f.readlines()]
            else:
                self.__hist_list = []
        self.__idx = len(self.__hist_list)

    def prev(self) -> str:
        self.__idx -= 1
        if self.__idx < 0:
            self.__idx = 0
        if len(self.__hist_list) == 0:
            return ""
        return self.__hist_list[self.__idx]

    def next(self) -> str:
        self.__idx += 1
        if self.__idx >= len(self.__hist_list):
            self.__idx = len(self.__hist_list)
            return ""
        return self.__hist_list[self.__idx]

    def append(self, cmd) -> None:
        if len(cmd) > 0 and (len(self.__hist_list) == 0 or self.__hist_list[-1] != cmd):
            self.__hist_list.append(cmd)
        self.__idx = len(self.__hist_list)

    def write(self) -> None:
        if len(self.__hist_list) == 0:
            return
        if len(self.__hist_list) > self.MAX_LEN:
            self.__hist_list = self.__hist_list[len(self.__hist_list) - self.MAX_LEN:]
        self.__hist_list = list(filter(lambda x: len(x.strip()) > 0, self.__hist_list))
        with open(".oterm_history.txt", "w") as f:
            f.write('\n'.join(self.__hist_list))


# class

class OTerm(npyscreen.Autocomplete):
    M_FUNCTION_BASE = re.compile(r"\.(?P<fname>\w+(?=\())\((?P<args>[\w\'\",=\s:/\[\]\{\}\(\)]+(?!$))?")
    M_FUNCTION_FILTER = re.compile(r"((\([^\(]+(?=[^\(]+\)).\))|(\[[^\[]+(?=[^\[]+\]).\]))|(\{[^\{]+(?=[^\{]+\}).\})")
    M_FUNCTION_SPLIT = re.compile(r"(?:(?:[\w_]+(?= *\=) *\= *)?(?:(?:\{(?=[^\{]+\})[^\}]+\})|(?:\((?=[^\(]+\))[^\)]+\))|(?:\[(?=[^\[]+\])[^\]]+\])|[\w_]+))")
    M_FUNCTION_ARGS = re.compile(r"(?P<key>\w+(?=\s*\=))?\s*\=?\s*(?P<arg>[,\w\'\"\s:/\[\]\{\}\(\)]+)")

    def __init__(self, *args, **keywords):
        super().__init__(*args, **keywords)
        self.crowracle = self.parent.parentApp.crowracle
        self.crowtology_functions = [(el_str, inspect.signature(el)) for el, el_str in [(getattr(self.crowracle, el_str), el_str) for el_str in filter(lambda x: not x.startswith("_"), dir(self.crowracle))] if callable(el)]
        self.history = self.parent.parentApp.history

    def set_up_handlers(self):
        super().set_up_handlers()
        self.handlers.update({
                curses.ascii.NL: self.process_command,
                curses.ascii.CR: self.process_command,
                curses.KEY_PPAGE: self.scroll_up,
                curses.KEY_NPAGE: self.scroll_down,
                curses.KEY_HOME: self.scroll_home,
                curses.KEY_END: self.scroll_end,
                curses.KEY_F1: self.call_for_help,
                curses.KEY_UP: self.history_up,
                curses.KEY_DOWN: self.history_down,
            })

    def history_up(self, input):
        self.value = self.history.prev()
        self.cursor_position = len(self.value)

    def history_down(self, input):
        self.value = self.history.next()
        self.cursor_position = len(self.value)

    def scroll_up(self, input):
        self.parent.wMain.start_display_at -= 1
        if self.parent.wMain.start_display_at < 0:
            self.parent.wMain.start_display_at = 0
        self.parent.wMain.update()

    def scroll_down(self, input):
        self.parent.wMain.start_display_at += 1
        if self.parent.wMain.start_display_at > len(self.parent.wMain.values) - len(self.parent.wMain._my_widgets):
            self.parent.wMain.start_display_at = len(self.parent.wMain.values) - len(self.parent.wMain._my_widgets)
        self.parent.wMain.update()

    def scroll_home(self, input):
        self.parent.wMain.start_display_at = 0
        self.parent.wMain.update()

    def scroll_end(self, input):
        self.parent.wMain.start_display_at = len(self.parent.wMain.values) - len(self.parent.wMain._my_widgets)
        self.parent.wMain.update()

    def process_function_values(self, value):
        return eval(value)  # TODO: maybe something better and safer in the future

    def parse_function(self, command):
        m = self.M_FUNCTION_BASE.match(command)
        if m is None:
            raise Exception("Could not split the function into 'function name' and 'args'! Is the signature correct? (enclosing brackets, etc.)")
        fname, arg_string = m.group("fname", "args")
        if fname is None:
            raise Exception("Could not retrieve function name from the command!")

        args = []
        kwargs = {}
        if arg_string is not None:
            arg_list = self.M_FUNCTION_SPLIT.findall(arg_string)
            for arg in arg_list:
                m = self.M_FUNCTION_ARGS.match(arg)
                if m is None:
                    raise Exception(f"Malformed argument: '{arg}'!")
                key, value = m.group("key", "arg")
                if key is None:
                    args.append(self.process_function_values(value))
                else:
                    kwargs[key] = self.process_function_values(value)

        return fname, args, kwargs

    def popup(self, message, title=""):
        if type(message) is list:
            message = list(chain.from_iterable([wrap(m, width=100) for m in message]))
        else:
            message = wrap(message)
        tmp_win = npyscreen.MessagePopup(name=title, framed=True, columns=100, lines=len(message) + 5)
        tmp_win.TextWidget.values = message
        tmp_win.TextWidget.editable = False
        tmp_win.display()
        tmp_win.edit()
        return ""

    def error_popup(self, message, title=""):
        self.popup(([title, ""] if len(title) > 0 else []) + (message if type(message) is list else [message]), title="Error")

    def display_result(self, function_call, result):
        if result is None:
            result = []
        else:
            def wrap_or_tap(value):
                if type(value) is list:
                    return value
                else:
                    return wrap(value, width=self.parent.columns - 10)

            if type(result) is list:
                result = list(chain.from_iterable([wrap_or_tap(m) for m in result]))
            else:
                result = wrap_or_tap(result)
            result = [""] + result

        self.parent.wMain.buffer([function_call] + result + ["#" * (self.parent.columns - 10)])
        self.parent.wMain.update()

    def process_command(self, *args, **keywords):
        try:
            if self.value.startswith("."):
                try:
                    fname, args, kwargs = self.parse_function(self.value)
                except BaseException as e:
                    self.error_popup([f"Error parsing command '{self.value}':", str(e)])
                else:
                    try:
                        func = getattr(self.crowracle, fname)
                    except AttributeError:
                        self.error_popup(f"Crowtology client has no function '{fname}'!", title=self.value)
                    else:
                        result = func(*args, **kwargs)
                        self.display_result(self.value, result)
            else:
                raise Exception("SPARQL execution not implemented yet!")
        except BaseException as e:
            self.error_popup([f"Error executing command '{self.value}':", str(e)])
        else:
            self.history.append(self.value)
        finally:
            self.value = ""

    def get_crowtology_functions(self, partial_text: str):
        result = [(f"{func_str}{str(signature)}", func_str + "(" + (")" if len(signature.parameters) == 0 else "")) for func_str, signature in filter(lambda x: x[0].startswith(partial_text), self.crowtology_functions)]
        choices, functions = zip(*result) if len(result) > 0 else ([], [])
        return choices, functions

    def call_for_help(self, input):
        if self.value.startswith("."):  # crowtology client function call
            end = self.value.index("(") if "(" in self.value else len(self.value)
            choices, func_strings = self.get_crowtology_functions(self.value[1:end])
            if len(choices) > 0:
                if len(choices) == 1:
                    fn = func_strings[0]
                    fn = fn[:fn.index("(")]
                    self.popup([choices[0], ""] + getattr(self.crowracle, fn).__doc__.split("\n"), title="Help")
                else:
                    self.auto_complete(None)
            else:
                self.error_popup("Unknown command, cannot display help!")
        else:  # ontology query
            choices = ["Abeceda", "Bbceda"]

    def auto_complete(self, input):
        if self.value.startswith("."):  # crowtology client function call
            choices, func_strings = self.get_crowtology_functions(self.value[1:])
            if len(choices) > 0:
                if len(choices) == 1:
                    choice_id = 0
                else:
                    choice_id = self.get_choice(choices)
                self.value = "." + func_strings[choice_id]
            else:
                curses.beep()
        else:  # ontology query
            choices = ["Abeceda", "Bbceda"]

        self.cursor_position=len(self.value)


class MainForm(npyscreen.FormMutt):
    MAIN_WIDGET_CLASS = npyscreen.BufferPager
    COMMAND_WIDGET_CLASS = OTerm

    def beforeEditing(self):
        self.keypress_timeout = 30
        self.wStatus1.value = "TAB: auto-complete command; F1: help; F5: show history; F8 clear history; PGDN/PGUP: scroll output; UP/DOWN scroll history"
        self.wStatus2.value = "Command (.<crowtology_function> | RDF triple query (e.g. '?obj :hasColor ?')):"
        self.wMain.editable = False
        self.wMain.interested_in_mouse_even_when_not_editable = True

    def while_waiting(self):
        self.parentApp.history.write()

class OntoTerm(npyscreen.NPSAppManaged):

    def __init__(self):
        super().__init__()
        npyscreen.BufferPager.DEFAULT_MAXLEN = 500
        # initialize the ontology client
        self.crowracle = CrowtologyClient()
        self.onto = self.crowracle.onto

        self.history = History()

    def onStart(self):
        self.addForm("MAIN", MainForm, name="OntoTerm")
        # self.addForm("HINTER", History, name="History")
        return super().onStart()

    def onCleanExit(self):
        self.history.write()
        return super().onCleanExit()


def main():
    ot = OntoTerm()
    try:
        ot.run()
    except KeyboardInterrupt:
        ot.history.write()
        print("User requested shutdown.")


if __name__ == '__main__':
    main()