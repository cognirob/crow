# This expresses a status send from nlp_node after processing a sentence to the visualizator
string det_obj # detected object in sentence
string det_command # detected command in sentence
string status # status of NLP processor: say something / processing / did not understand
bool found_in_ws # if the detected object is in the workspace or not
