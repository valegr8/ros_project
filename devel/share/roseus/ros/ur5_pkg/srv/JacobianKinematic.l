;; Auto-generated. Do not edit!


(when (boundp 'ur5_pkg::JacobianKinematic)
  (if (not (find-package "UR5_PKG"))
    (make-package "UR5_PKG"))
  (shadow 'JacobianKinematic (find-package "UR5_PKG")))
(unless (find-package "UR5_PKG::JACOBIANKINEMATIC")
  (make-package "UR5_PKG::JACOBIANKINEMATIC"))
(unless (find-package "UR5_PKG::JACOBIANKINEMATICREQUEST")
  (make-package "UR5_PKG::JACOBIANKINEMATICREQUEST"))
(unless (find-package "UR5_PKG::JACOBIANKINEMATICRESPONSE")
  (make-package "UR5_PKG::JACOBIANKINEMATICRESPONSE"))

(in-package "ROS")





(defclass ur5_pkg::JacobianKinematicRequest
  :super ros::object
  :slots (_input ))

(defmethod ur5_pkg::JacobianKinematicRequest
  (:init
   (&key
    ((:input __input) 0.0)
    )
   (send-super :init)
   (setq _input (float __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; float64 _input
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _input
       (sys::poke _input (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _input
     (setq _input (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur5_pkg::JacobianKinematicResponse
  :super ros::object
  :slots (_output ))

(defmethod ur5_pkg::JacobianKinematicResponse
  (:init
   (&key
    ((:output __output) 0.0)
    )
   (send-super :init)
   (setq _output (float __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; float64 _output
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _output
       (sys::poke _output (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _output
     (setq _output (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur5_pkg::JacobianKinematic
  :super ros::object
  :slots ())

(setf (get ur5_pkg::JacobianKinematic :md5sum-) "6c59364dede48a4429627e3e0efa7049")
(setf (get ur5_pkg::JacobianKinematic :datatype-) "ur5_pkg/JacobianKinematic")
(setf (get ur5_pkg::JacobianKinematic :request) ur5_pkg::JacobianKinematicRequest)
(setf (get ur5_pkg::JacobianKinematic :response) ur5_pkg::JacobianKinematicResponse)

(defmethod ur5_pkg::JacobianKinematicRequest
  (:response () (instance ur5_pkg::JacobianKinematicResponse :init)))

(setf (get ur5_pkg::JacobianKinematicRequest :md5sum-) "6c59364dede48a4429627e3e0efa7049")
(setf (get ur5_pkg::JacobianKinematicRequest :datatype-) "ur5_pkg/JacobianKinematicRequest")
(setf (get ur5_pkg::JacobianKinematicRequest :definition-)
      "float64 input
---
float64 output
")

(setf (get ur5_pkg::JacobianKinematicResponse :md5sum-) "6c59364dede48a4429627e3e0efa7049")
(setf (get ur5_pkg::JacobianKinematicResponse :datatype-) "ur5_pkg/JacobianKinematicResponse")
(setf (get ur5_pkg::JacobianKinematicResponse :definition-)
      "float64 input
---
float64 output
")



(provide :ur5_pkg/JacobianKinematic "6c59364dede48a4429627e3e0efa7049")


