;; Auto-generated. Do not edit!


(when (boundp 'ur5_pkg::InverseKinematic)
  (if (not (find-package "UR5_PKG"))
    (make-package "UR5_PKG"))
  (shadow 'InverseKinematic (find-package "UR5_PKG")))
(unless (find-package "UR5_PKG::INVERSEKINEMATIC")
  (make-package "UR5_PKG::INVERSEKINEMATIC"))
(unless (find-package "UR5_PKG::INVERSEKINEMATICREQUEST")
  (make-package "UR5_PKG::INVERSEKINEMATICREQUEST"))
(unless (find-package "UR5_PKG::INVERSEKINEMATICRESPONSE")
  (make-package "UR5_PKG::INVERSEKINEMATICRESPONSE"))

(in-package "ROS")





(defclass ur5_pkg::InverseKinematicRequest
  :super ros::object
  :slots (_x _y _z ))

(defmethod ur5_pkg::InverseKinematicRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur5_pkg::InverseKinematicResponse
  :super ros::object
  :slots (_theta0 _theta1 _theta2 _theta3 _theta4 _theta5 ))

(defmethod ur5_pkg::InverseKinematicResponse
  (:init
   (&key
    ((:theta0 __theta0) 0.0)
    ((:theta1 __theta1) 0.0)
    ((:theta2 __theta2) 0.0)
    ((:theta3 __theta3) 0.0)
    ((:theta4 __theta4) 0.0)
    ((:theta5 __theta5) 0.0)
    )
   (send-super :init)
   (setq _theta0 (float __theta0))
   (setq _theta1 (float __theta1))
   (setq _theta2 (float __theta2))
   (setq _theta3 (float __theta3))
   (setq _theta4 (float __theta4))
   (setq _theta5 (float __theta5))
   self)
  (:theta0
   (&optional __theta0)
   (if __theta0 (setq _theta0 __theta0)) _theta0)
  (:theta1
   (&optional __theta1)
   (if __theta1 (setq _theta1 __theta1)) _theta1)
  (:theta2
   (&optional __theta2)
   (if __theta2 (setq _theta2 __theta2)) _theta2)
  (:theta3
   (&optional __theta3)
   (if __theta3 (setq _theta3 __theta3)) _theta3)
  (:theta4
   (&optional __theta4)
   (if __theta4 (setq _theta4 __theta4)) _theta4)
  (:theta5
   (&optional __theta5)
   (if __theta5 (setq _theta5 __theta5)) _theta5)
  (:serialization-length
   ()
   (+
    ;; float64 _theta0
    8
    ;; float64 _theta1
    8
    ;; float64 _theta2
    8
    ;; float64 _theta3
    8
    ;; float64 _theta4
    8
    ;; float64 _theta5
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta0
       (sys::poke _theta0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta1
       (sys::poke _theta1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta2
       (sys::poke _theta2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta3
       (sys::poke _theta3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta4
       (sys::poke _theta4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta5
       (sys::poke _theta5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta0
     (setq _theta0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta1
     (setq _theta1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta2
     (setq _theta2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta3
     (setq _theta3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta4
     (setq _theta4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta5
     (setq _theta5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur5_pkg::InverseKinematic
  :super ros::object
  :slots ())

(setf (get ur5_pkg::InverseKinematic :md5sum-) "c7202f6e227428a04da015ed4eae1b79")
(setf (get ur5_pkg::InverseKinematic :datatype-) "ur5_pkg/InverseKinematic")
(setf (get ur5_pkg::InverseKinematic :request) ur5_pkg::InverseKinematicRequest)
(setf (get ur5_pkg::InverseKinematic :response) ur5_pkg::InverseKinematicResponse)

(defmethod ur5_pkg::InverseKinematicRequest
  (:response () (instance ur5_pkg::InverseKinematicResponse :init)))

(setf (get ur5_pkg::InverseKinematicRequest :md5sum-) "c7202f6e227428a04da015ed4eae1b79")
(setf (get ur5_pkg::InverseKinematicRequest :datatype-) "ur5_pkg/InverseKinematicRequest")
(setf (get ur5_pkg::InverseKinematicRequest :definition-)
      "float64 x
float64 y
float64 z
---
float64 theta0
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
")

(setf (get ur5_pkg::InverseKinematicResponse :md5sum-) "c7202f6e227428a04da015ed4eae1b79")
(setf (get ur5_pkg::InverseKinematicResponse :datatype-) "ur5_pkg/InverseKinematicResponse")
(setf (get ur5_pkg::InverseKinematicResponse :definition-)
      "float64 x
float64 y
float64 z
---
float64 theta0
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
")



(provide :ur5_pkg/InverseKinematic "c7202f6e227428a04da015ed4eae1b79")


