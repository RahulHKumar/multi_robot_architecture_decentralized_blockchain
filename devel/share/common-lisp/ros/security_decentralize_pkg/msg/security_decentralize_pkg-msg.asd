
(cl:in-package :asdf)

(defsystem "security_decentralize_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "block1" :depends-on ("_package_block1"))
    (:file "_package_block1" :depends-on ("_package"))
    (:file "block2" :depends-on ("_package_block2"))
    (:file "_package_block2" :depends-on ("_package"))
    (:file "block3" :depends-on ("_package_block3"))
    (:file "_package_block3" :depends-on ("_package"))
    (:file "custom_1" :depends-on ("_package_custom_1"))
    (:file "_package_custom_1" :depends-on ("_package"))
    (:file "custom_2" :depends-on ("_package_custom_2"))
    (:file "_package_custom_2" :depends-on ("_package"))
    (:file "custom_3" :depends-on ("_package_custom_3"))
    (:file "_package_custom_3" :depends-on ("_package"))
    (:file "malicious_flags" :depends-on ("_package_malicious_flags"))
    (:file "_package_malicious_flags" :depends-on ("_package"))
    (:file "pp1" :depends-on ("_package_pp1"))
    (:file "_package_pp1" :depends-on ("_package"))
    (:file "pp2" :depends-on ("_package_pp2"))
    (:file "_package_pp2" :depends-on ("_package"))
    (:file "pp3" :depends-on ("_package_pp3"))
    (:file "_package_pp3" :depends-on ("_package"))
  ))