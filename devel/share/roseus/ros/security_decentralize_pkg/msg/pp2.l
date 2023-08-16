;; Auto-generated. Do not edit!


(when (boundp 'security_decentralize_pkg::pp2)
  (if (not (find-package "SECURITY_DECENTRALIZE_PKG"))
    (make-package "SECURITY_DECENTRALIZE_PKG"))
  (shadow 'pp2 (find-package "SECURITY_DECENTRALIZE_PKG")))
(unless (find-package "SECURITY_DECENTRALIZE_PKG::PP2")
  (make-package "SECURITY_DECENTRALIZE_PKG::PP2"))

(in-package "ROS")
;;//! \htmlinclude pp2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass security_decentralize_pkg::pp2
  :super ros::object
  :slots (_robot_name ))

(defmethod security_decentralize_pkg::pp2
  (:init
   (&key
    ((:robot_name __robot_name) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _robot_name __robot_name)
   self)
  (:robot_name
   (&rest __robot_name)
   (if (keywordp (car __robot_name))
       (send* _robot_name __robot_name)
     (progn
       (if __robot_name (setq _robot_name (car __robot_name)))
       _robot_name)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _robot_name
    (send _robot_name :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _robot_name
       (send _robot_name :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _robot_name
     (send _robot_name :deserialize buf ptr-) (incf ptr- (send _robot_name :serialization-length))
   ;;
   self)
  )

(setf (get security_decentralize_pkg::pp2 :md5sum-) "a714a45eb282cdb6b20eb8ead90c5b9a")
(setf (get security_decentralize_pkg::pp2 :datatype-) "security_decentralize_pkg/pp2")
(setf (get security_decentralize_pkg::pp2 :definition-)
      "std_msgs/String robot_name

================================================================================
MSG: std_msgs/String
string data

")



(provide :security_decentralize_pkg/pp2 "a714a45eb282cdb6b20eb8ead90c5b9a")


