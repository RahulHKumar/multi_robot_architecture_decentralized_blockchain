; Auto-generated. Do not edit!


(cl:in-package security_decentralize_pkg-msg)


;//! \htmlinclude malicious_flags.msg.html

(cl:defclass <malicious_flags> (roslisp-msg-protocol:ros-message)
  ((bot1_mal
    :reader bot1_mal
    :initarg :bot1_mal
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (bot2_mal
    :reader bot2_mal
    :initarg :bot2_mal
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (bot3_mal
    :reader bot3_mal
    :initarg :bot3_mal
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass malicious_flags (<malicious_flags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <malicious_flags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'malicious_flags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name security_decentralize_pkg-msg:<malicious_flags> is deprecated: use security_decentralize_pkg-msg:malicious_flags instead.")))

(cl:ensure-generic-function 'bot1_mal-val :lambda-list '(m))
(cl:defmethod bot1_mal-val ((m <malicious_flags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:bot1_mal-val is deprecated.  Use security_decentralize_pkg-msg:bot1_mal instead.")
  (bot1_mal m))

(cl:ensure-generic-function 'bot2_mal-val :lambda-list '(m))
(cl:defmethod bot2_mal-val ((m <malicious_flags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:bot2_mal-val is deprecated.  Use security_decentralize_pkg-msg:bot2_mal instead.")
  (bot2_mal m))

(cl:ensure-generic-function 'bot3_mal-val :lambda-list '(m))
(cl:defmethod bot3_mal-val ((m <malicious_flags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:bot3_mal-val is deprecated.  Use security_decentralize_pkg-msg:bot3_mal instead.")
  (bot3_mal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <malicious_flags>) ostream)
  "Serializes a message object of type '<malicious_flags>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bot1_mal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bot2_mal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bot3_mal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <malicious_flags>) istream)
  "Deserializes a message object of type '<malicious_flags>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bot1_mal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bot2_mal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bot3_mal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<malicious_flags>)))
  "Returns string type for a message object of type '<malicious_flags>"
  "security_decentralize_pkg/malicious_flags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'malicious_flags)))
  "Returns string type for a message object of type 'malicious_flags"
  "security_decentralize_pkg/malicious_flags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<malicious_flags>)))
  "Returns md5sum for a message object of type '<malicious_flags>"
  "1fbd4377f4fa1942d302cc06fa5d8c8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'malicious_flags)))
  "Returns md5sum for a message object of type 'malicious_flags"
  "1fbd4377f4fa1942d302cc06fa5d8c8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<malicious_flags>)))
  "Returns full string definition for message of type '<malicious_flags>"
  (cl:format cl:nil "std_msgs/String bot1_mal~%std_msgs/String bot2_mal~%std_msgs/String bot3_mal~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'malicious_flags)))
  "Returns full string definition for message of type 'malicious_flags"
  (cl:format cl:nil "std_msgs/String bot1_mal~%std_msgs/String bot2_mal~%std_msgs/String bot3_mal~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <malicious_flags>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bot1_mal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bot2_mal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bot3_mal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <malicious_flags>))
  "Converts a ROS message object to a list"
  (cl:list 'malicious_flags
    (cl:cons ':bot1_mal (bot1_mal msg))
    (cl:cons ':bot2_mal (bot2_mal msg))
    (cl:cons ':bot3_mal (bot3_mal msg))
))
