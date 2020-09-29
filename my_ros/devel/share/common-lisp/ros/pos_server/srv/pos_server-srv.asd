
(cl:in-package :asdf)

(defsystem "pos_server-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pos_service" :depends-on ("_package_pos_service"))
    (:file "_package_pos_service" :depends-on ("_package"))
  ))