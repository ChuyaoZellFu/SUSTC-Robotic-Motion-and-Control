; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteProtectionZone-request.msg.html

(cl:defclass <DeleteProtectionZone-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ProtectionZoneHandle
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneHandle)))
)

(cl:defclass DeleteProtectionZone-request (<DeleteProtectionZone-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteProtectionZone-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteProtectionZone-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteProtectionZone-request> is deprecated: use kortex_driver-srv:DeleteProtectionZone-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteProtectionZone-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteProtectionZone-request>) ostream)
  "Serializes a message object of type '<DeleteProtectionZone-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteProtectionZone-request>) istream)
  "Deserializes a message object of type '<DeleteProtectionZone-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteProtectionZone-request>)))
  "Returns string type for a service object of type '<DeleteProtectionZone-request>"
  "kortex_driver/DeleteProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteProtectionZone-request)))
  "Returns string type for a service object of type 'DeleteProtectionZone-request"
  "kortex_driver/DeleteProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteProtectionZone-request>)))
  "Returns md5sum for a message object of type '<DeleteProtectionZone-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteProtectionZone-request)))
  "Returns md5sum for a message object of type 'DeleteProtectionZone-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteProtectionZone-request>)))
  "Returns full string definition for message of type '<DeleteProtectionZone-request>"
  (cl:format cl:nil "ProtectionZoneHandle input~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteProtectionZone-request)))
  "Returns full string definition for message of type 'DeleteProtectionZone-request"
  (cl:format cl:nil "ProtectionZoneHandle input~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteProtectionZone-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteProtectionZone-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteProtectionZone-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteProtectionZone-response.msg.html

(cl:defclass <DeleteProtectionZone-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteProtectionZone-response (<DeleteProtectionZone-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteProtectionZone-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteProtectionZone-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteProtectionZone-response> is deprecated: use kortex_driver-srv:DeleteProtectionZone-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteProtectionZone-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteProtectionZone-response>) ostream)
  "Serializes a message object of type '<DeleteProtectionZone-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteProtectionZone-response>) istream)
  "Deserializes a message object of type '<DeleteProtectionZone-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteProtectionZone-response>)))
  "Returns string type for a service object of type '<DeleteProtectionZone-response>"
  "kortex_driver/DeleteProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteProtectionZone-response)))
  "Returns string type for a service object of type 'DeleteProtectionZone-response"
  "kortex_driver/DeleteProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteProtectionZone-response>)))
  "Returns md5sum for a message object of type '<DeleteProtectionZone-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteProtectionZone-response)))
  "Returns md5sum for a message object of type 'DeleteProtectionZone-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteProtectionZone-response>)))
  "Returns full string definition for message of type '<DeleteProtectionZone-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteProtectionZone-response)))
  "Returns full string definition for message of type 'DeleteProtectionZone-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteProtectionZone-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteProtectionZone-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteProtectionZone-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteProtectionZone)))
  'DeleteProtectionZone-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteProtectionZone)))
  'DeleteProtectionZone-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteProtectionZone)))
  "Returns string type for a service object of type '<DeleteProtectionZone>"
  "kortex_driver/DeleteProtectionZone")