static pointer CLO350();
static pointer CLO355();
static pointer (*ftab[3])();

#define QUOTE_STRINGS_SIZE 25
static char *quote_strings[QUOTE_STRINGS_SIZE]={
    "#(:additional-weight-list)",
    "(:rleg :lleg)",
    "send-message",
    "super",
    ":inverse-kinematics",
    ":additional-weight-list",
    "#(:additional-weight-list)",
    "(:rleg :lleg)",
    ":fullbody-inverse-kinematics",
    ":toe-p",
    ":child-link",
    "\"package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts\"",
    "load",
    "\"package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils\"",
    ":entry",
    "\"___hrp3hand_utils\"",
    ":init-org",
    "hrp2jsknts-robot",
    ":methods",
    "assoc",
    ":init",
    "defmethod",
    "get-hrp2-with-hand-class-methods",
    "\"(self class target-coords &rest args &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg))) &allow-other-keys)\"",
    "\"(self class target-coords &rest args &key (additional-weight-list (mapcar #'(lambda (l) (list (send self l :toe-p :child-link) 0)) '(:rleg :lleg))) &allow-other-keys)\"",
  };
