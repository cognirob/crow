# This expresses a program send from nlp_node after processing a sentence as a list of lists of keys and values

std_msgs/Header header                  # timestamp in the header is the acquisition time and frame

StringList[] keys                  # sentence program as string
StringList[] values                  # sentence program as string
