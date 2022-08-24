
(cl:in-package :asdf)

(defsystem "ros_exercises-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "primes_list" :depends-on ("_package_primes_list"))
    (:file "_package_primes_list" :depends-on ("_package"))
  ))