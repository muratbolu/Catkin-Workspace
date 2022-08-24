
(cl:in-package :asdf)

(defsystem "ros_exercises-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "check_palindrome" :depends-on ("_package_check_palindrome"))
    (:file "_package_check_palindrome" :depends-on ("_package"))
    (:file "compute_statistics" :depends-on ("_package_compute_statistics"))
    (:file "_package_compute_statistics" :depends-on ("_package"))
    (:file "find_min_array" :depends-on ("_package_find_min_array"))
    (:file "_package_find_min_array" :depends-on ("_package"))
  ))