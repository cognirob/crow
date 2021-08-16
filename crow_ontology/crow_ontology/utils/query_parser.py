import re
from typing import Tuple


class QueryParser():
    qmatcher = re.compile(r"\s*(?P<selector>\*)?\s*(?P<ns>\w+(?=\.))?\.?(?P<core>[\w-]+)?\s*(?P<transitor>\+|-)?\s*")

    def __parse_query(self, query: str) -> Tuple[str, str]:
        select = ''
        where = ''
        for part, ptype in zip(query.split(","), ["s", "p", "o"]):
            m = self.qmatcher.match(part)
            if m is None:
                raise Exception(f"Malformed query!\nFull query: {query}\nErroneous part: {part}")
            gdict = m.groupdict()


    def __parse_filter(self, qfilter: str) -> str:
        return ""

    def parse(self, query: str) -> str:
        query, *filters = map(str.strip, query.split(";"))
        if len(query) > 0:
            select, where = self.__parse_query(query)
            if len(filters) > 0:
                filter_strings = [self.__parse_filter(qf) for qf in filters]
                for fs in filter_strings:
                    if len(fs) > 0:
                        where += fs
            wstr = '\n'.join(where)
            qstr = f"""SELECT {select}
            WHERE {{
                {wstr}
            }}
            """
            return qstr
        else:
            raise Exception("Empty query!")


