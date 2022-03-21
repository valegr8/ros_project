;; Auto-generated. Do not edit!


(when (boundp 'ur5_pkg::ForwardKinematic)
  (if (not (find-package "UR5_PKG"))
    (make-package "UR5_PKG"))
  (shadow 'ForwardKinematic (find-package "UR5_PKG")))
(unless (find-package "UR5_PKG::FORWARDKINEMATIC")
  (make-package "UR5_PKG::FORWARDKINEMATIC"))
(unless (find-package "UR5_PKG::FORWARDKINEMATICREQUEST")
  (make-package "UR5_PKG::FORWARDKINEMATICREQUEST"))
(unless (find-package "UR5_PKG::FORWARDKINEMATICRESPONSE")
  (make-package "UR5_PKG::FORWARDKINEMATICRESPONSE"))

(in-package "ROS")





(defclass ur5_pkg::ForwardKinematicRequest
  :super ros::object
  :slots (_theta0 _theta1 _theta2 _theta3 _theta4 _theta5 ))

(defmethod ur5_pkg::ForwardKinematicRequest
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

(defclass ur5_pkg::ForwardKinematicResponse
  :super ros::object
  :slots (_x _y _z ))

(defmethod ur5_pkg::ForwardKinematicResponse
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

(defclass ur5_pkg::ForwardKinematic
  :super ros::object
  :slots ())

(setf (get ur5_pkg::ForwardKinematic :md5sum-) "1d89c8f7ccd238d24ca1c2a794f9dbfe")
(setf (get ur5_pkg::ForwardKinematic :datatype-) "ur5_pkg/ForwardKinematic")
(setf (get ur5_pkg::ForwardKinematic :request) ur5_pkg::ForwardKinematicRequest)
(setf (get ur5_pkg::ForwardKinematic :response) ur5_pkg::ForwardKinematicResponse)

(defmethod ur5_pkg::ForwardKinematicRequest
  (:response () (instance ur5_pkg::ForwardKinematicResponse :init)))

(setf (get ur5_pkg::ForwardKinematicRequest :md5sum-) "1d89c8f7ccd238d24ca1c2a794f9dbfe")
(setf (get ur5_pkg::ForwardKinematicRequest :datatype-) "ur5_pkg/ForwardKinematicRequest")
(setf (get ur5_pkg::ForwardKinematicRequest :definition-)
      "float64 theta0
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
---
float64 x
float64 y
float64 z
")

(setf (get ur5_pkg::ForwardKinematicResponse :md5sum-) "1d89c8f7ccd238d24ca1c2a794f9dbfe")
(setf (get ur5_pkg::ForwardKinematicResponse :datatype-) "ur5_pkg/ForwardKinematicResponse")
(setf (get ur5_pkg::ForwardKinematicResponse :definition-)
      "float64 theta0
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
---
float64 x
float64 y
float64 z
")



(provide :ur5_pkg/ForwardKinematic "1d89c8f7ccd238d24ca1c2a794f9dbfe")


