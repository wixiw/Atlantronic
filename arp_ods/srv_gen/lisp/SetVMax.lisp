; Auto-generated. Do not edit!


(cl:in-package arp_ods-srv)


;//! \htmlinclude SetVMax-request.msg.html

(cl:defclass <SetVMax-request> (roslisp-msg-protocol:ros-message)
  ((vMax
    :reader vMax
    :initarg :vMax
    :type cl:float
    :initform 0.0)
   (setToDefault
    :reader setToDefault
    :initarg :setToDefault
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVMax-request (<SetVMax-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVMax-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVMax-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arp_ods-srv:<SetVMax-request> is deprecated: use arp_ods-srv:SetVMax-request instead.")))

(cl:ensure-generic-function 'vMax-val :lambda-list '(m))
(cl:defmethod vMax-val ((m <SetVMax-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_ods-srv:vMax-val is deprecated.  Use arp_ods-srv:vMax instead.")
  (vMax m))

(cl:ensure-generic-function 'setToDefault-val :lambda-list '(m))
(cl:defmethod setToDefault-val ((m <SetVMax-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arp_ods-srv:setToDefault-val is deprecated.  Use arp_ods-srv:setToDefault instead.")
  (setToDefault m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVMax-request>) ostream)
  "Serializes a message object of type '<SetVMax-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'setToDefault) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVMax-request>) istream)
  "Deserializes a message object of type '<SetVMax-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vMax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'setToDefault) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVMax-request>)))
  "Returns string type for a service object of type '<SetVMax-request>"
  "arp_ods/SetVMaxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVMax-request)))
  "Returns string type for a service object of type 'SetVMax-request"
  "arp_ods/SetVMaxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVMax-request>)))
  "Returns md5sum for a message object of type '<SetVMax-request>"
  "84953976940388b2a4dbffc8e04d3822")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVMax-request)))
  "Returns md5sum for a message object of type 'SetVMax-request"
  "84953976940388b2a4dbffc8e04d3822")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVMax-request>)))
  "Returns full string definition for message of type '<SetVMax-request>"
  (cl:format cl:nil "float64 vMax~%bool setToDefault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVMax-request)))
  "Returns full string definition for message of type 'SetVMax-request"
  (cl:format cl:nil "float64 vMax~%bool setToDefault~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVMax-request>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVMax-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVMax-request
    (cl:cons ':vMax (vMax msg))
    (cl:cons ':setToDefault (setToDefault msg))
))
;//! \htmlinclude SetVMax-response.msg.html

(cl:defclass <SetVMax-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetVMax-response (<SetVMax-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVMax-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVMax-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arp_ods-srv:<SetVMax-response> is deprecated: use arp_ods-srv:SetVMax-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVMax-response>) ostream)
  "Serializes a message object of type '<SetVMax-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVMax-response>) istream)
  "Deserializes a message object of type '<SetVMax-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVMax-response>)))
  "Returns string type for a service object of type '<SetVMax-response>"
  "arp_ods/SetVMaxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVMax-response)))
  "Returns string type for a service object of type 'SetVMax-response"
  "arp_ods/SetVMaxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVMax-response>)))
  "Returns md5sum for a message object of type '<SetVMax-response>"
  "84953976940388b2a4dbffc8e04d3822")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVMax-response)))
  "Returns md5sum for a message object of type 'SetVMax-response"
  "84953976940388b2a4dbffc8e04d3822")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVMax-response>)))
  "Returns full string definition for message of type '<SetVMax-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVMax-response)))
  "Returns full string definition for message of type 'SetVMax-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVMax-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVMax-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVMax-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVMax)))
  'SetVMax-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVMax)))
  'SetVMax-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVMax)))
  "Returns string type for a service object of type '<SetVMax>"
  "arp_ods/SetVMax")
