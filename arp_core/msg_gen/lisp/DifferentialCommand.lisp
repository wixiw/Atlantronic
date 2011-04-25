; Auto-generated. Do not edit!


(cl:in-package arp_core-msg)


;//! \htmlinclude DifferentialCommand.msg.html

(cl:defclass <DifferentialCommand> (roslisp-msg-protocol:ros-message)
  ((v_left
    :reader v_left
    :initarg :v_left
    :type cl:float
    :initform 0.0)
   (v_right
    :reader v_right
    :initarg :v_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass DifferentialCommand (<DifferentialCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DifferentialCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DifferentialCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arp_core-msg:<DifferentialCommand> is deprecated: use arp_core-msg:DifferentialCommand instead.")))

(cl:ensure-generic-function 'v_left-val :lambda-list '(m))
(cl:defmethod v_left-val ((m <DifferentialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_core-msg:v_left-val is deprecated.  Use arp_core-msg:v_left instead.")
  (v_left m))

(cl:ensure-generic-function 'v_right-val :lambda-list '(m))
(cl:defmethod v_right-val ((m <DifferentialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_core-msg:v_right-val is deprecated.  Use arp_core-msg:v_right instead.")
  (v_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DifferentialCommand>) ostream)
  "Serializes a message object of type '<DifferentialCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DifferentialCommand>) istream)
  "Deserializes a message object of type '<DifferentialCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DifferentialCommand>)))
  "Returns string type for a message object of type '<DifferentialCommand>"
  "arp_core/DifferentialCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DifferentialCommand)))
  "Returns string type for a message object of type 'DifferentialCommand"
  "arp_core/DifferentialCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DifferentialCommand>)))
  "Returns md5sum for a message object of type '<DifferentialCommand>"
  "50fa646698267f553e03f81238667677")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DifferentialCommand)))
  "Returns md5sum for a message object of type 'DifferentialCommand"
  "50fa646698267f553e03f81238667677")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DifferentialCommand>)))
  "Returns full string definition for message of type '<DifferentialCommand>"
  (cl:format cl:nil "float64 v_left~%float64 v_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DifferentialCommand)))
  "Returns full string definition for message of type 'DifferentialCommand"
  (cl:format cl:nil "float64 v_left~%float64 v_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DifferentialCommand>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DifferentialCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'DifferentialCommand
    (cl:cons ':v_left (v_left msg))
    (cl:cons ':v_right (v_right msg))
))
