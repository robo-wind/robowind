#ifndef OBJECT_CLASSES_
#define OBJECT_CLASSES_

#include <vector>
#include <map>
using namespace std;

static const std::map<std::string, int> BUTTON_LABEL_ID_MAP = 
                                                            {
                                                              std::make_pair ("first_visit",0),
                                                              std::make_pair ("honorary_citizen",1),
                                                              std::make_pair ("happy_birthday",2),
                                                              std::make_pair ("celebrating",3),
                                                              std::make_pair ("happily_ever_after",4),
                                                              std::make_pair ("just_graduated",5),
                                                              std::make_pair ("plaza_inn",6)
                                                            };

static const std::map<std::string, int> RPS_LABEL_ID_MAP = 
                                                            {
                                                              std::make_pair ("rock",0),
                                                              std::make_pair ("paper",1),
                                                              std::make_pair ("scissors",2),
                                                            };


static const std::map<std::string, int> RALPH_OBJECT_LABEL_ID_MAP = 
                                                            {
                                                              std::make_pair ("medal",0),
                                                              std::make_pair ("underwear",1),
                                                              std::make_pair ("crown",2),
                                                              std::make_pair ("cookie",3),
                                                            };

#endif