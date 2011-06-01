
(cl:in-package :asdf)

(defsystem "arp_ods-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetVMax" :depends-on ("_package_SetVMax"))
    (:file "_package_SetVMax" :depends-on ("_package"))
  ))