; Auto-generated. Do not edit!


(cl:in-package arp_core-msg)


;//! \htmlinclude Start.msg.html

(cl:defclass <Start> (roslisp-msg-protocol:ros-message)
  ((go
    :reader go
    :initarg :go
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Start (<Start>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arp_core-msg:<Start> is deprecated: use arp_core-msg:Start instead.")))

(cl:ensure-generic-function 'go-val :lambda-list '(m))
(cl:defmethod go-val ((m <Start>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_core-msg:go-val is deprecated.  Use arp_core-msg:go instead.")
  (go m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start>) ostream)
  "Serializes a message object of type '<Start>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'go)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start>) istream)
  "Deserializes a message object of type '<Start>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'go)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start>)))
  "Returns string type for a message object of type '<Start>"
  "arp_core/Start")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start)))
  "Returns string type for a message object of type 'Start"
  "arp_core/Start")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start>)))
  "Returns md5sum for a message object of type '<Start>"
  "cde602380b04a9f9d04072dff04b244b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start)))
  "Returns md5sum for a message object of type 'Start"
  "cde602380b04a9f9d04072dff04b244b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start>)))
  "Returns full string definition for message of type '<Start>"
  (cl:format cl:nil "uint8 go~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start)))
  "Returns full string definition for message of type 'Start"
  (cl:format cl:nil "uint8 go~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start>))
  "Converts a ROS message object to a list"
  (cl:list 'Start
    (cl:cons ':go (go msg))
))
