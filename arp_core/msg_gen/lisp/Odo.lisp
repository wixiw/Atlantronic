; Auto-generated. Do not edit!


(cl:in-package arp_core-msg)


;//! \htmlinclude Odo.msg.html

(cl:defclass <Odo> (roslisp-msg-protocol:ros-message)
  ((odo_left
    :reader odo_left
    :initarg :odo_left
    :type cl:float
    :initform 0.0)
   (odo_right
    :reader odo_right
    :initarg :odo_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass Odo (<Odo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Odo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Odo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arp_core-msg:<Odo> is deprecated: use arp_core-msg:Odo instead.")))

(cl:ensure-generic-function 'odo_left-val :lambda-list '(m))
(cl:defmethod odo_left-val ((m <Odo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_core-msg:odo_left-val is deprecated.  Use arp_core-msg:odo_left instead.")
  (odo_left m))

(cl:ensure-generic-function 'odo_right-val :lambda-list '(m))
(cl:defmethod odo_right-val ((m <Odo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_core-msg:odo_right-val is deprecated.  Use arp_core-msg:odo_right instead.")
  (odo_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Odo>) ostream)
  "Serializes a message object of type '<Odo>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'odo_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'odo_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Odo>) istream)
  "Deserializes a message object of type '<Odo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odo_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odo_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Odo>)))
  "Returns string type for a message object of type '<Odo>"
  "arp_core/Odo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Odo)))
  "Returns string type for a message object of type 'Odo"
  "arp_core/Odo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Odo>)))
  "Returns md5sum for a message object of type '<Odo>"
  "90d73fc2677028867971b4ef8a5770ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Odo)))
  "Returns md5sum for a message object of type 'Odo"
  "90d73fc2677028867971b4ef8a5770ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Odo>)))
  "Returns full string definition for message of type '<Odo>"
  (cl:format cl:nil "float64 odo_left~%float64 odo_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Odo)))
  "Returns full string definition for message of type 'Odo"
  (cl:format cl:nil "float64 odo_left~%float64 odo_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Odo>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Odo>))
  "Converts a ROS message object to a list"
  (cl:list 'Odo
    (cl:cons ':odo_left (odo_left msg))
    (cl:cons ':odo_right (odo_right msg))
))
