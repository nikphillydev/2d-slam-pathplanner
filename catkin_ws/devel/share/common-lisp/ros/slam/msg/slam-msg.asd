
(cl:in-package :asdf)

(defsystem "slam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DoubleOccupancyGrid" :depends-on ("_package_DoubleOccupancyGrid"))
    (:file "_package_DoubleOccupancyGrid" :depends-on ("_package"))
  ))