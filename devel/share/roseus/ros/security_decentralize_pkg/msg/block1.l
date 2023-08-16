;; Auto-generated. Do not edit!


(when (boundp 'security_decentralize_pkg::block1)
  (if (not (find-package "SECURITY_DECENTRALIZE_PKG"))
    (make-package "SECURITY_DECENTRALIZE_PKG"))
  (shadow 'block1 (find-package "SECURITY_DECENTRALIZE_PKG")))
(unless (find-package "SECURITY_DECENTRALIZE_PKG::BLOCK1")
  (make-package "SECURITY_DECENTRALIZE_PKG::BLOCK1"))

(in-package "ROS")
;;//! \htmlinclude block1.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass security_decentralize_pkg::block1
  :super ros::object
  :slots (_robot_hash ))

(defmethod security_decentralize_pkg::block1
  (:init
   (&key
    ((:robot_hash __robot_hash) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _robot_hash __robot_hash)
   self)
  (:robot_hash
   (&rest __robot_hash)
   (if (keywordp (car __robot_hash))
       (send* _robot_hash __robot_hash)
     (progn
       (if __robot_hash (setq _robot_hash (car __robot_hash)))
       _robot_hash)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _robot_hash
    (send _robot_hash :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _robot_hash
       (send _robot_hash :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _robot_hash
     (send _robot_hash :deserialize buf ptr-) (incf ptr- (send _robot_hash :serialization-length))
   ;;
   self)
  )

(setf (get security_decentralize_pkg::block1 :md5sum-) "df6427b3d0603bcc36155fb56a25758c")
(setf (get security_decentralize_pkg::block1 :datatype-) "security_decentralize_pkg/block1")
(setf (get security_decentralize_pkg::block1 :definition-)
      "std_msgs/String robot_hash

================================================================================
MSG: std_msgs/String
string data

")



(provide :security_decentralize_pkg/block1 "df6427b3d0603bcc36155fb56a25758c")


