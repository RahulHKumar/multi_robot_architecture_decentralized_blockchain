; Auto-generated. Do not edit!


(cl:in-package security_decentralize_pkg-msg)


;//! \htmlinclude pp1.msg.html

(cl:defclass <pp1> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass pp1 (<pp1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pp1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pp1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name security_decentralize_pkg-msg:<pp1> is deprecated: use security_decentralize_pkg-msg:pp1 instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <pp1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader security_decentralize_pkg-msg:robot_name-val is deprecated.  Use security_decentralize_pkg-msg:robot_name instead.")
  (robot_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pp1>) ostream)
  "Serializes a message object of type '<pp1>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_name) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pp1>) istream)
  "Deserializes a message object of type '<pp1>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_name) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pp1>)))
  "Returns string type for a message object of type '<pp1>"
  "security_decentralize_pkg/pp1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pp1)))
  "Returns string type for a message object of type 'pp1"
  "security_decentralize_pkg/pp1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pp1>)))
  "Returns md5sum for a message object of type '<pp1>"
  "a714a45eb282cdb6b20eb8ead90c5b9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pp1)))
  "Returns md5sum for a message object of type 'pp1"
  "a714a45eb282cdb6b20eb8ead90c5b9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pp1>)))
  "Returns full string definition for message of type '<pp1>"
  (cl:format cl:nil "std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pp1)))
  "Returns full string definition for message of type 'pp1"
  (cl:format cl:nil "std_msgs/String robot_name~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pp1>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pp1>))
  "Converts a ROS message object to a list"
  (cl:list 'pp1
    (cl:cons ':robot_name (robot_name msg))
))
