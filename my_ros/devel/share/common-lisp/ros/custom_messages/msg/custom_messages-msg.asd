
(cl:in-package :asdf)

(defsystem "custom_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Two" :depends-on ("_package_Two"))
    (:file "_package_Two" :depends-on ("_package"))
  ))