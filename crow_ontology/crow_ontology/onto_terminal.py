from crow_ontology.utils import query_parser
import sys
import time
from typing import Any, Dict, List, Set, Tuple
from textwrap import wrap

import pandas as pd
import npyscreen
from crow_ontology.crowracle_client import CrowtologyClient
import curses
import inspect
import re
from itertools import chain
import os
from crow_ontology.utils import QueryParser
from npyscreen.wgmultiline import MultiLine
import rclpy
from typing import Union
from rdflib.plugins.sparql.processor import prepareQuery



class FileLog():
    """A simple logging class that can store the log into a file.
    """

    def __init__(self, log_file=".oterm_errors.txt", max_len=100) -> None:
        self.LOG_FILE = log_file
        self.MAX_LEN = max_len
        if not os.path.isfile(self.LOG_FILE):
            with open(self.LOG_FILE, "w") as f:
                pass

        with open(self.LOG_FILE, "r+") as f:
            if f.readable():
                self._list = [h.strip() for h in f.readlines()]
            else:
                self._list = []
        self._has_changes = False

    def __append_single(self, msg) -> None:
        if len(msg) > 0:
            self._list.append(msg)

    def append(self, msg: Union[str, List[str]]) -> None:
        """Appends the provided message to the log.

        Args:
            msg (Union[str, List[str]]): Message to append.
        """

        if type(msg) is list:
            self._list += msg
        else:
            self.__append_single(msg)
        self._has_changes = True

    def delete(self):
        """Deletes the current log content. Does not immediately delete the log file!
        """
        self._list = []
        self._has_changes = True

    @property
    def list(self):
        return self._list

    def write(self) -> None:
        """Writes the current log content to the file.
        """
        if len(self._list) == 0:
            return
        if len(self._list) > self.MAX_LEN:
            self._list = self._list[len(self._list) - self.MAX_LEN:]
        self._list = list(filter(lambda x: len(x.strip()) > 0, self._list))
        with open(self.LOG_FILE, "w") as f:
            f.write('\n'.join(self._list))
        self._has_changes = False


class History(FileLog):
    MAX_LEN = 100

    def __init__(self, history_file=".oterm_history.txt") -> None:
        super().__init__(log_file=history_file)
        self.__idx = len(self._list)

    def prev(self) -> str:
        self.__idx -= 1
        if self.__idx < 0:
            self.__idx = 0
        if len(self._list) == 0:
            return ""
        return self._list[self.__idx]

    def next(self) -> str:
        self.__idx += 1
        if self.__idx >= len(self._list):
            self.__idx = len(self._list)
            return ""
        return self._list[self.__idx]

    def append(self, cmd) -> None:
        if len(cmd) > 0 and (len(self._list) == 0 or self._list[-1] != cmd):
            self._list.append(cmd)
            self._has_changes = True
        self.__idx = len(self._list)


class ActionSelector(npyscreen.MultiSelectAction):
    _contained_widgets = npyscreen.BoxBasic

    def actionHighlighted(self, act_on_this, key_press):
        self.parent.on_ok(value=act_on_this)


class Histeresis(npyscreen.ActionFormV2):
    """Form to disply the complete history (from the 'History' logging object).
    """

    def create(self):
        self.selector = self.add(ActionSelector, name="History")
        self.history = self.parentApp.history

    def beforeEditing(self):
        self.selector.values = self.history.list

    def set_up_handlers(self):
        super().set_up_handlers()
        self.handlers.update({
                curses.ascii.ESC: self.on_cancel,
                curses.KEY_EXIT: self.on_cancel
        })

    def on_ok(self, inp=None, value=None):
        if value is not None:
            self.parentApp.getForm('MAIN').wCommand.value = value
        self.parentApp.switchFormPrevious()

    def on_cancel(self, inp=None):
        self.parentApp.switchFormPrevious()


class QueryDoc(npyscreen.Form):
    """A form to display the help/documentation from the query_parser.py file.
    """

    def create(self):
        self.qdoc = self.add(MultiLine, name="QDoc")

    def beforeEditing(self):
        self.qdoc.values = query_parser.__doc__.split("\n")

    def afterEditing(self):
        self.parentApp.switchFormPrevious()


class OTerm(npyscreen.Autocomplete):
    """The main control widget that also serves as a command line for the main form.
    """
    M_FUNCTION_BASE = re.compile(r"\.(?P<fname>\w+(?=\())\((?P<args>[\w\'\",=\s:\/\[\]\{\}\(\)\.]+(?!$))?", flags=re.UNICODE)
    M_FUNCTION_FILTER = re.compile(r"((\([^\(]+(?=[^\(]+\)).\))|(\[[^\[]+(?=[^\[]+\]).\]))|(\{[^\{]+(?=[^\{]+\}).\})", flags=re.UNICODE)
    M_FUNCTION_SPLIT = re.compile(r"(?:\s*(?:(?:[\w_\s]+)(?=\=) *\= *)?(?:(?:\{(?=[^\{]+\})[^\}]+\})|(?:\((?=[^\(]+\))[^\)]+\))|(?:\[(?=[^\[]+\])[^\]]+\])|[\'\"\w_\s\t\n]+))", flags=re.UNICODE)
    M_FUNCTION_ARGS = re.compile(r"(?P<key>\w+(?=\s*\=))?\s*\=?\s*(?P<arg>[,\w\'\"\s:/\[\]\{\}\(\)\.]+)", flags=re.UNICODE)
    P_SPO = re.compile(r"^(?P<modifier>(?:!|\+|-|(!\?))(?= ))?\s*(?P<s>[\?:\w]+)?(?P<p>\s+[\?:\w]+)?(?P<o>\s+[\?:\w]+)?", flags=re.UNICODE)
    P_NS = re.compile(r"^ *(?P<ns>\w+(?=:))", flags=re.UNICODE)

    def __init__(self, *args, **keywords) -> None:
        super().__init__(*args, **keywords)
        # reference to crowracle
        self.crowracle = self.parent.parentApp.crowracle
        self.crowtology_functions = [(el_str, inspect.signature(el)) for el, el_str in [(getattr(self.crowracle, el_str), el_str) for el_str in filter(lambda x: not x.startswith("_"), dir(self.crowracle))] if callable(el)]
        # logging object references
        self.history = self.parent.parentApp.history
        self.error_log = self.parent.parentApp.error_log
        # query parser instance
        self.query_parser = QueryParser(self.crowracle.CROW)
        # get namespaces for URI shortening and expansion
        self.onto_namespaces = self.crowracle.onto.namespaces
        if "crow" not in self.onto_namespaces.keys():
            self.onto_namespaces["crow"] = self.crowracle.CROW
        self.replacement_ns = {}
        for ns, uri in self.onto_namespaces.items():
            if ns == "base":
                continue
            uri = str(uri)
            self.replacement_ns[uri] = ns + ":"
            if uri.endswith("#"):
                self.replacement_ns[uri[:-1] + "/"] = ns + ":"
            if uri.endswith("/"):
                self.replacement_ns[uri[:-1] + "#"] = ns + ":"

    def set_up_handlers(self):
        super().set_up_handlers()
        self.handlers.update({
                curses.ascii.NL: self.process_command,
                curses.ascii.CR: self.process_command,
                curses.KEY_PPAGE: self.scroll_up,
                curses.KEY_NPAGE: self.scroll_down,
                # curses.KEY_NPAGE + curses.ascii.ALT: self.test,
                curses.KEY_HOME: self.scroll_home,
                curses.KEY_END: self.scroll_end,
                curses.KEY_F1: self.call_for_help,
                curses.KEY_F2: self.show_docs,
                curses.KEY_F5: self.show_history,
                curses.KEY_F8: self.delete_history,
                curses.KEY_F10: self.exit_application,
                curses.KEY_UP: self.history_up,
                curses.KEY_DOWN: self.history_down,
                "^s": self.help_subject,
                "^p": self.help_property,
                "^o": self.help_object,
            })

    def show_docs(self, input):
        self.parent.parentApp.switchForm('QUERYDOC')

    def show_history(self, input):
        self.parent.parentApp.switchForm('HISTORY')

    def delete_history(self, input):
        decision = npyscreen.notify_ok_cancel("Are you sure you want to delete all history?", title="History deletion confirmation", form_color="DANGER")
        if decision:
            self.history.delete()
        return ""

    def exit_application(self, input=None):
        self.parent.parentApp.setNextForm(None)
        self.parent.editing = False
        self.editing = False
        print("Goodbye!")

    def test(self):
        print("LAL")

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

    def process_function_values(self, value: str) -> Any:
        """Get arguments from a string (e.g. "12", "'aaa'", "[1, 2, 3]", ...)

        Args:
            value ([str]): A string containing function a argument.

        Returns:
            [Any]: A python value/object (e.g. list, dict, etc.)
        """
        return eval(value)  # TODO: maybe something better and safer in the future

    def parse_function(self, command: str) -> Tuple[str, list, dict]:
        """Parses a string from user input containing a full function call,
        i.e., function name and signature with args/kwargs.

        Args:
            command (str): The string with a function call.

        Raises:
            Exception: Error when parsing or executing the function.

        Returns:
            Tuple[str, list, dict]: The function name, arguments and keyword arguments.
        """
        m = self.M_FUNCTION_BASE.match(command)  # first, split the function into name & signature
        if m is None:
            raise Exception("Could not split the function into 'function name' and 'args'! Is the signature correct? (enclosing brackets, etc.)")
        fname, arg_string = m.group("fname", "args")
        if fname is None:
            raise Exception("Could not retrieve function name from the command!")

        # next, process args
        args = []
        kwargs = {}
        if arg_string is not None:
            arg_list = self.M_FUNCTION_SPLIT.findall(arg_string)  # split individual args
            for arg in arg_list:  # for each arg
                m = self.M_FUNCTION_ARGS.match(arg.strip())  # try to split it into key: val pair or just val
                if m is None:
                    raise Exception(f"Malformed argument: '{arg}'!")
                key, value = m.group("key", "arg")
                try:
                    arg_val = self.process_function_values(value)
                except BaseException as e:
                    raise Exception(f"Error while evaluating function argument(s): '{arg}'!\nError was:\n{e}")
                else:
                    if key is None:
                        args.append(arg_val)
                    else:
                        kwargs[key] = arg_val

        return fname, args, kwargs

    def popup(self, message: Union[str, List[str]], title: str="", form_color: str="STANDOUT") -> None:
        """Displays a popup message.

        Args:
            message ([str, list]): The message do display.
            title (str, optional): Title of the window. Defaults to "".
            form_color (str, optional): Color scheme to use. Use "DANGER" for errors. Defaults to "STANDOUT".
        """
        if type(message) is list:
            message = list(chain.from_iterable([wrap(m, width=100) for m in message]))
        else:
            message = wrap(message)
        decision = npyscreen.notify_confirm(message, title=title, wide=True, editw=1, form_color="DANGER")
        self.error_log.append(message)

    def error_popup(self, message: Union[str, List[str]], title: str="") -> None:
        """Displays an error message. Se OTerm.display_popup for details on arguments.

        Args:
            message (Union[str, List[str]])
            title (str, optional): Defaults to "".
        """
        self.popup(([title, "\n"] if len(title) > 0 else []) + (message if type(message) is list else message.split("\n")), title="Error", form_color="DANGER")

    def display_result(self, function_call: str, result: Union[None, List, Dict, Set, str], info: str="") -> None:
        """Displays a function result into the main form buffer.

        Args:
            function_call (str): The original user input (for clarity on what returned the result).
            result (Union[None, Any]): The result of the function, if any.
            info (str, optional): Additional info, like execution time, etc. Defaults to "".
        """
        if result is None:
            result = []
        else:
            # Some helper functions to process the input
            def ravel_dict(d: dict) -> List[str]:
                return [f"{k}: {v}" for k, v in d.items()]

            def wrap_or_tap(value) -> List[str]:
                if type(value) in [list, set]:
                    return value
                elif type(value) is dict:
                    return ravel_dict(value)
                else:
                    return wrap(value, width=self.parent.columns - 10)

            if type(result) in [list, set]:
                result = list(chain.from_iterable([wrap_or_tap(m) for m in result]))
            elif type(result) is dict:
                result = ravel_dict(result)
            else:
                result = wrap_or_tap(result)
            result = ["\n"] + result

        if len(info) > 0:
            result = [info] + result
        self.parent.wMain.buffer([function_call] + result + ["#" * (self.parent.columns - 10)])
        self.parent.wMain.update()

    def _shorten_uri(self, input_uri: str) -> str:
        """Replaces the namespace part of the URI with a short namespace name.

        Args:
            input_uri (str): The URI to shorten.

        Returns:
            str: Shortened version of the URI.
        """
        for uri, ns in self.replacement_ns.items():
            input_uri = input_uri.replace(uri, ns)
        return input_uri

    def process_command(self, input=None):
        if len(self.value) == 0:
            return
        try:
            if self.value.startswith("."):  # CrowracleClient function call:
                try:
                    fname, args, kwargs = self.parse_function(self.value)
                except BaseException as e:
                    self.error_popup([f"Error parsing command '{self.value}':"] + str(e).split("\n"))
                    return
                try:
                    func = getattr(self.crowracle, fname)
                except AttributeError:
                    self.error_popup(f"Crowtology client has no function '{fname}'!", title=self.value)
                    return
                start = time.time()
                result = func(*args, **kwargs)
                run_time = time.time() - start
                self.display_result(self.value, result, info=f"[function executed in {run_time:.5f} seconds]")
            else:  # Ontology SPARQL query:
                try:  # try to parse the query
                    query_string = self.query_parser.parse(self.value)
                except BaseException as e:
                    self.error_popup([f"Error parsing query '{self.value}':"] + str(e).split("\n"))
                    return
                try:  # try to prepare the query
                    start = time.time()
                    query = prepareQuery(query_string, initNs=self.onto_namespaces)
                    run_time_qp = time.time() - start
                except BaseException as e:
                    self.error_popup([f"Error preparing query '{self.value}':"] + str(e).split("\n") + ["\n\nThe parsed query string:\n"] + query_string.split("\n"))
                    return
                start = time.time()
                result = self.crowracle.onto.query(query)  # execute the query
                run_time_ask = time.time() - start
                if len(result) > 0:
                    vnames = [str(v) for v in result.vars]
                    records = [{v: self._shorten_uri(r[v]) for v in vnames} for r in result]
                    df = pd.DataFrame.from_records(records)
                    self.display_result(self.value, df.to_markdown().split("\n"),
                        info=f"[query prepared in {run_time_qp:.5f} seconds and evaluated in {run_time_ask:.5f} seconds]")
                else:
                    self.display_result(self.value, "> query returned empty result <")
        except BaseException as e:
            self.error_popup([f"Error executing command '{self.value}':"] + str(e).split("\n"))
        else:
            self.history.append(self.value)
        finally:
            self.value = ""

    def get_crowtology_functions(self, partial_text: str):
        result = [(f"{func_str}{str(signature)}", func_str + "(" + (")" if len(signature.parameters) == 0 else "")) for func_str, signature in filter(lambda x: x[0].startswith(partial_text), self.crowtology_functions)]
        choices, functions = zip(*result) if len(result) > 0 else ([], [])
        return choices, functions

    def call_for_help(self, input=None):
        if self.value.startswith("."):  # crowtology client function call
            end = self.value.index("(") if "(" in self.value else len(self.value)
            choices, func_strings = self.get_crowtology_functions(self.value[1:end])
            if len(choices) > 0:
                if len(choices) == 1:
                    fn = func_strings[0]
                    self.value = "." + fn
                    self.cursor_position = len(self.value)
                    fn = fn[:fn.index("(")]
                    self.popup([choices[0], ""] + getattr(self.crowracle, fn).__doc__.split("\n"), title="Help")
                else:
                    self.auto_complete()
            else:
                self.error_popup("Unknown command, cannot display help!")
        else:  # ontology query
            choices = "This is still not implemented :(".split(" ")

    def _expand_ns(self, elem: str) -> str:
        m = self.P_NS.match(elem)
        if m is None:
            return elem
        ns = m.group("ns")
        if ns is None or ns not in self.onto_namespaces:
            return elem
        return elem.replace(ns + ":", self.onto_namespaces[ns])

    def _filter_entities(self, query_func, partial_text=""):
        expanded_text = self._expand_ns(partial_text)
        return list(filter(lambda x: expanded_text in str(x), set(query_func(None, None))))

    def help_subject(self, partial_text=""):
        return self._filter_entities(self.crowracle.onto.subjects, partial_text)

    def help_property(self, partial_text=""):
        return self._filter_entities(self.crowracle.onto.predicates, partial_text)

    def help_object(self, partial_text=""):
        return self._filter_entities(self.crowracle.onto.objects, partial_text)

    def auto_complete(self, input=None):
        if len(self.value) == 0:
            return

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
            seq = self.value
            inp = seq[max(seq.rindex(".") + 1 if "." in seq else 0, seq.rindex(";") + 1 if ";" in seq else 0, seq.rindex("{") + 1 if "{" in seq else 0):].strip()
            m = self.P_SPO.match(inp)
            if m is None:
                self.error_popup("Could not recognize subject, property, or object. Try invoking completion manually, i.e. CTRL+s, CTRL+p, CTRL+o.")
                curses.beep()
                return
            s, p, o = m.group("s", "p", "o")
            part = ""
            if o is None:
                if p is None:
                    if s is None:
                        choices = self.help_subject()
                    else:
                        part = s.strip()
                        choices = self.help_subject(partial_text=part)
                else:
                    part = p.strip()
                    choices = self.help_property(partial_text=part)
            else:
                part = o.strip()
                choices = self.help_object(partial_text=part)
            if len(choices) > 0:
                if len(choices) == 1:
                    choice_id = 0
                else:
                    choice_id = self.get_choice(choices)
                choice = self._shorten_uri(choices[choice_id])[::-1]
                part = part[::-1]
                inp = inp[::-1]
                self.value = seq[::-1].replace(inp, inp.replace(part, choice, 1) if len(part) > 0 else choice + inp, 1)[::-1]
            else:
                curses.beep()

        self.cursor_position = len(self.value)


class MainForm(npyscreen.FormMutt):
    MAIN_WIDGET_CLASS = npyscreen.BufferPager
    COMMAND_WIDGET_CLASS = OTerm

    def beforeEditing(self):
        self.keypress_timeout = 30
        self.wStatus1.value = "TAB: auto-complete command | F1: context help | F2: SPARQL help | F5: show history | F8 clear history | F10: quit | PGDN/PGUP: scroll output | UP/DOWN: history prev/next"
        self.wStatus2.value = "Command (.<crowtology_function> | SPARQL query):"
        self.wMain.editable = False
        # self.wMain.interested_in_mouse_even_when_not_editable = True

    def while_waiting(self):
        self.parentApp.history.write()
        self.parentApp.error_log.write()

class OntoTerm(npyscreen.NPSAppManaged):

    def __init__(self):
        super().__init__()
        npyscreen.BufferPager.DEFAULT_MAXLEN = 500
        npyscreen.Popup.DEFAULT_COLUMNS = 100
        npyscreen.PopupWide.DEFAULT_LINES = 20
        npyscreen.PopupWide.SHOW_ATY = 1
        # initialize the ontology client
        self.crowracle = CrowtologyClient()

        self.onto = self.crowracle.onto
        self.crowracle.node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

        self.history = History()
        self.error_log = FileLog()

    def onStart(self):
        # npyscreen.setTheme(npyscreen.Themes.TransparentThemeLightText)
        self.addForm("MAIN", MainForm, name="OntoTerm")
        self.addForm("HISTORY", Histeresis, name="History")
        self.addForm("QUERYDOC", QueryDoc, name="Ontology query documentation")
        return super().onStart()

    def onCleanExit(self):
        self.history.write()
        self.error_log.write()
        print("Exited cleanly")
        return super().onCleanExit()


def main():
    ot = OntoTerm()
    try:
        ot.run()
    except KeyboardInterrupt:
        ot.switchForm(None)
        ot.history.write()
        ot.error_log.write()
        print("User requested shutdown.")


if __name__ == '__main__':
    main()
