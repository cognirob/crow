import re
from typing import Tuple

__doc__ = """
## Basic query
    Currently, or parts of a full query must be separated by " . " (a dot with a space around it).
    This must be used to terminate every triple, filter, union, etc.!

    Basic triplet query:
        <subject> <predicate> <object> .
        or using variables
        ?s ?p ?o .
    Example:
        ?obj :hasColor :COLOR_GREEN
        Translates into SPARQL query (assuming "CROW" is base namespace):

        SELECT ?obj
        WHERE {
            ?obj CROW:hasColor CROW:COLOR_GREEN .
        }
        The query will return a list (substitution for "?obj") of objects
        with green color.

        "?obj :hasColor ?"
        Translates into SPARQL query:

        SELECT ?obj ?element1
        WHERE {
            ?obj CROW:hasColor ?element1 .
        }
        The query will return two lists - for ?obj (some entity)
        and ?element1 (colors, likely). Naming the first element is actually
        not necessary. It is useful only in complex queries, i.e.:
        "?obj :hasColor ?, ?obj a :CUBE_HOLES"
        Translates into SPARQL query:

        SELECT ?obj ?element1
        WHERE {
            ?obj CROW:hasColor ?element1 ,
            ?obj a :CUBE_HOLES .
        }
        Returns objects that have color and are of type "CROW:CUBE_HOLES"
        (will also return their colors).

## Syntax
    'a' - syntax sugar for RDF:type (e.g., ?obj a :some_class)
    '?some_name' - named return element/variable
    '?' - unnamed return element/variable
    '?_' - variable that should not be included in the result

## Blank nodes
    [ :p "v" ] :q "w" .
        - [] - creates a blank node that can be used in the triple.
    _:b57 :p "v" .
    _:b57 :q "w" .
        - does the same thing as the example above.
        - in this case, blank node is created explicitely (_:b57 -> with label "b57")

## Predicate-object lists
    queries separated by ';' share the same subject, e.g.:
        ?x  foaf:name  ?name ; foaf:mbox  ?mbox .
    is the same as:
        ?x  foaf:name  ?name .
        ?x  foaf:mbox  ?mbox .

## Object lists
    queries separated by ',' share the same subject and predicate, e.g.:
        ?x foaf:nick  "Alice" , "Alice_" .
    is the same as:
        ?x  foaf:nick  "Alice" .
        ?x  foaf:nick  "Alice_" .

## Filtering and advanced query grouping
    FILTER
        basic filter
            usage:
                ! (?price < 30)
            (query = FILTER(?price < 30))
            operators:
                <,>,=,!=
        regex
            usage:
                ! (?name, "Smith")
            (query = FILTER regex(?name, "Smith"))
        NOT EXIST
            usage:
                ! ?nan rdfs:subClassOf ?cls .
            (query = FILTER NOT EXISTS { ?nan rdfs:subClassOf ?cls })

        EXISTS
            usage:
                !? ?nan rdfs:subClassOf ?cls .
            (query = EXISTS {?nan rdfs:subClassOf ?cls .})
            Tests if such triple is present in the databases but does not generate bindings.

        Nested filters are currently unsupported.
    MINUS
        usage:
            - {?s :p1 ?v2 }
        (query = MINUS {?s :p1 ?v2 } )
   MINUS
     { ?x ?y ?z }
    OPTIONAL
        usage:
            + ?x  foaf:mbox  ?mbox .
        (query = OPTIONAL { ?x  foaf:mbox  ?mbox })
    UNION
        usage:
            { ?book dc10:title  ?title } | { ?book dc11:title  ?title }
        (query = { ?book dc10:title  ?title } UNION { ?book dc11:title  ?title })
        - either of the alternatives can match

# Property Paths
    iri	PredicatePath	An IRI. A path of length one.
    ^elt	InversePath	Inverse path (object to subject).
    elt1 / elt2	SequencePath	A sequence path of elt1 followed by elt2.
    elt1 | elt2	AlternativePath	A alternative path of elt1 or elt2 (all possibilities are tried).
    elt*	ZeroOrMorePath	A path that connects the subject and object of the path by zero or more matches of elt.
    elt+	OneOrMorePath	A path that connects the subject and object of the path by one or more matches of elt.
    elt?	ZeroOrOnePath	A path that connects the subject and object of the path by zero or one matches of elt.

"""


class QueryParser():
    qmatcher = re.compile(r"\s*(?P<selector>\*)?\s*(?P<ns>\w+(?=\.))?\.?(?P<core>[\w-]+)?\s*(?P<transitor>\+|-)?\s*")
    P_SPLITTER = re.compile(r"(?P<modifier>(?:!|\+|-|(!\?))(?= ))?(?P<core>(?:.(?!\.[ $]))+) \.(?:\s|$)", flags=re.UNICODE)
    P_VARMOD = re.compile(r"\?(?<=[^|\s|\^])(?P<ignore>_)?(?P<vname>\w+)?", flags=re.UNICODE)
    P_UNION = re.compile(r"(?<=\} )\|(?= \{)", flags=re.UNICODE)
    P_FILT_REGEX = re.compile(r"\([\w\?]+\s*,\s*.+\)", flags=re.UNICODE)
    P_FILT_COMP = re.compile(r"\(.+\)", flags=re.UNICODE)

    def __init__(self, default_ns="") -> None:
        self.default_ns = f"PREFIX : <{default_ns}>\n" if len(default_ns) > 0 else ""
        self.selected = []
        self.autogen_idx = 0

    def _parse_single(self, core: str, modifier: str=None) -> str:
        core = core.strip()
        tmp_core = ""
        last_end = 0
        # Replace and add variables
        for varm in self.P_VARMOD.finditer(core):
            v_ignore, vname = varm.group("ignore", "vname")
            if vname is None:
                vname = f"?autogen{self.autogen_idx}"
                tmp_core += core[last_end:varm.span()[0]] + vname
                last_end = varm.span()[1]
            else:
                vname = "?" + vname
            if v_ignore is None and vname not in self.selected:
                self.selected.append(vname)
            self.autogen_idx += 1
        if len(tmp_core) > 0:
            core = tmp_core + core[last_end:]
        # UNION operators
        core = self.P_UNION.sub("UNION", core)

        if modifier is None:
            return core + " ."
        else:
            if modifier == "!":
                fmod = ""
                if self.P_FILT_COMP.search(core) is None:
                    fmod = " NOT EXISTS { "
                    core += " }"
                else:
                    if self.P_FILT_REGEX.search(core) is not None:
                        fmod = " regex"
                core = "FILTER" + fmod + core
            elif modifier == "!?":
                core = "FILTER EXISTS { " + core + " }"
            elif modifier == "+":
                core = "OPTIONAL { " + core + " }"
            elif modifier == "-":
                core = "MINUS { " + core + " }"
            else:
                raise Exception(f"Unknown modifier '{modifier}'!")
            return core

    def parse(self, query: str) -> str:
        where = []
        self.selected = []
        self.autogen_idx = 0
        if query.strip().endswith("."):
            for m in self.P_SPLITTER.finditer(query):
                modifier, core = m.group("modifier",  "core")
                where.append(self._parse_single(core, modifier=modifier))
        else:
            try:
                where.append(self._parse_single(query))
            except BaseException as e:
                raise Exception(f"There was no dot at the end of the query, so I tried to match a single triplet but failed because:\n{e}")
        wheres = "\n".join(where)
        query = f"""{self.default_ns}SELECT {' '.join(self.selected)}
WHERE {{
    {wheres}
}}
"""
        return query
