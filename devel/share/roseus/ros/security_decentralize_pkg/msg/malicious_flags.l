;; Auto-generated. Do not edit!


(when (boundp 'security_decentralize_pkg::malicious_flags)
  (if (not (find-package "SECURITY_DECENTRALIZE_PKG"))
    (make-package "SECURITY_DECENTRALIZE_PKG"))
  (shadow 'malicious_flags (find-package "SECURITY_DECENTRALIZE_PKG")))
(unless (find-package "SECURITY_DECENTRALIZE_PKG::MALICIOUS_FLAGS")
  (make-package "SECURITY_DECENTRALIZE_PKG::MALICIOUS_FLAGS"))

(in-package "ROS")
;;//! \htmlinclude malicious_flags.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass security_decentralize_pkg::malicious_flags
  :super ros::object
  :slots (_bot1_mal _bot2_mal _bot3_mal ))

(defmethod security_decentralize_pkg::malicious_flags
  (:init
   (&key
    ((:bot1_mal __bot1_mal) (instance std_msgs::String :init))
    ((:bot2_mal __bot2_mal) (instance std_msgs::String :init))
    ((:bot3_mal __bot3_mal) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _bot1_mal __bot1_mal)
   (setq _bot2_mal __bot2_mal)
   (setq _bot3_mal __bot3_mal)
   self)
  (:bot1_mal
   (&rest __bot1_mal)
   (if (keywordp (car __bot1_mal))
       (send* _bot1_mal __bot1_mal)
     (progn
       (if __bot1_mal (setq _bot1_mal (car __bot1_mal)))
       _bot1_mal)))
  (:bot2_mal
   (&rest __bot2_mal)
   (if (keywordp (car __bot2_mal))
       (send* _bot2_mal __bot2_mal)
     (progn
       (if __bot2_mal (setq _bot2_mal (car __bot2_mal)))
       _bot2_mal)))
  (:bot3_mal
   (&rest __bot3_mal)
   (if (keywordp (car __bot3_mal))
       (send* _bot3_mal __bot3_mal)
     (progn
       (if __bot3_mal (setq _bot3_mal (car __bot3_mal)))
       _bot3_mal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _bot1_mal
    (send _bot1_mal :serialization-length)
    ;; std_msgs/String _bot2_mal
    (send _bot2_mal :serialization-length)
    ;; std_msgs/String _bot3_mal
    (send _bot3_mal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _bot1_mal
       (send _bot1_mal :serialize s)
     ;; std_msgs/String _bot2_mal
       (send _bot2_mal :serialize s)
     ;; std_msgs/String _bot3_mal
       (send _bot3_mal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _bot1_mal
     (send _bot1_mal :deserialize buf ptr-) (incf ptr- (send _bot1_mal :serialization-length))
   ;; std_msgs/String _bot2_mal
     (send _bot2_mal :deserialize buf ptr-) (incf ptr- (send _bot2_mal :serialization-length))
   ;; std_msgs/String _bot3_mal
     (send _bot3_mal :deserialize buf ptr-) (incf ptr- (send _bot3_mal :serialization-length))
   ;;
   self)
  )

(setf (get security_decentralize_pkg::malicious_flags :md5sum-) "1fbd4377f4fa1942d302cc06fa5d8c8f")
(setf (get security_decentralize_pkg::malicious_flags :datatype-) "security_decentralize_pkg/malicious_flags")
(setf (get security_decentralize_pkg::malicious_flags :definition-)
      "std_msgs/String bot1_mal
std_msgs/String bot2_mal
std_msgs/String bot3_mal

================================================================================
MSG: std_msgs/String
string data

")



(provide :security_decentralize_pkg/malicious_flags "1fbd4377f4fa1942d302cc06fa5d8c8f")


