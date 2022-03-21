
(cl:in-package :asdf)

(defsystem "ur5_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ForwardKinematic" :depends-on ("_package_ForwardKinematic"))
    (:file "_package_ForwardKinematic" :depends-on ("_package"))
    (:file "InverseKinematic" :depends-on ("_package_InverseKinematic"))
    (:file "_package_InverseKinematic" :depends-on ("_package"))
    (:file "JacobianKinematic" :depends-on ("_package_JacobianKinematic"))
    (:file "_package_JacobianKinematic" :depends-on ("_package"))
  ))