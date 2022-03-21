; Auto-generated. Do not edit!


(cl:in-package ur5_pkg-srv)


;//! \htmlinclude JacobianKinematic-request.msg.html

(cl:defclass <JacobianKinematic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:float
    :initform 0.0))
)

(cl:defclass JacobianKinematic-request (<JacobianKinematic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JacobianKinematic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JacobianKinematic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur5_pkg-srv:<JacobianKinematic-request> is deprecated: use ur5_pkg-srv:JacobianKinematic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <JacobianKinematic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur5_pkg-srv:input-val is deprecated.  Use ur5_pkg-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JacobianKinematic-request>) ostream)
  "Serializes a message object of type '<JacobianKinematic-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JacobianKinematic-request>) istream)
  "Deserializes a message object of type '<JacobianKinematic-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JacobianKinematic-request>)))
  "Returns string type for a service object of type '<JacobianKinematic-request>"
  "ur5_pkg/JacobianKinematicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JacobianKinematic-request)))
  "Returns string type for a service object of type 'JacobianKinematic-request"
  "ur5_pkg/JacobianKinematicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JacobianKinematic-request>)))
  "Returns md5sum for a message object of type '<JacobianKinematic-request>"
  "6c59364dede48a4429627e3e0efa7049")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JacobianKinematic-request)))
  "Returns md5sum for a message object of type 'JacobianKinematic-request"
  "6c59364dede48a4429627e3e0efa7049")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JacobianKinematic-request>)))
  "Returns full string definition for message of type '<JacobianKinematic-request>"
  (cl:format cl:nil "float64 input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JacobianKinematic-request)))
  "Returns full string definition for message of type 'JacobianKinematic-request"
  (cl:format cl:nil "float64 input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JacobianKinematic-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JacobianKinematic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JacobianKinematic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude JacobianKinematic-response.msg.html

(cl:defclass <JacobianKinematic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:float
    :initform 0.0))
)

(cl:defclass JacobianKinematic-response (<JacobianKinematic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JacobianKinematic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JacobianKinematic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur5_pkg-srv:<JacobianKinematic-response> is deprecated: use ur5_pkg-srv:JacobianKinematic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <JacobianKinematic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur5_pkg-srv:output-val is deprecated.  Use ur5_pkg-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JacobianKinematic-response>) ostream)
  "Serializes a message object of type '<JacobianKinematic-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JacobianKinematic-response>) istream)
  "Deserializes a message object of type '<JacobianKinematic-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JacobianKinematic-response>)))
  "Returns string type for a service object of type '<JacobianKinematic-response>"
  "ur5_pkg/JacobianKinematicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JacobianKinematic-response)))
  "Returns string type for a service object of type 'JacobianKinematic-response"
  "ur5_pkg/JacobianKinematicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JacobianKinematic-response>)))
  "Returns md5sum for a message object of type '<JacobianKinematic-response>"
  "6c59364dede48a4429627e3e0efa7049")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JacobianKinematic-response)))
  "Returns md5sum for a message object of type 'JacobianKinematic-response"
  "6c59364dede48a4429627e3e0efa7049")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JacobianKinematic-response>)))
  "Returns full string definition for message of type '<JacobianKinematic-response>"
  (cl:format cl:nil "float64 output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JacobianKinematic-response)))
  "Returns full string definition for message of type 'JacobianKinematic-response"
  (cl:format cl:nil "float64 output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JacobianKinematic-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JacobianKinematic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JacobianKinematic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JacobianKinematic)))
  'JacobianKinematic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JacobianKinematic)))
  'JacobianKinematic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JacobianKinematic)))
  "Returns string type for a service object of type '<JacobianKinematic>"
  "ur5_pkg/JacobianKinematic")