; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteMap-request.msg.html

(cl:defclass <DeleteMap-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle)))
)

(cl:defclass DeleteMap-request (<DeleteMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteMap-request> is deprecated: use kortex_driver-srv:DeleteMap-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteMap-request>) ostream)
  "Serializes a message object of type '<DeleteMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteMap-request>) istream)
  "Deserializes a message object of type '<DeleteMap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteMap-request>)))
  "Returns string type for a service object of type '<DeleteMap-request>"
  "kortex_driver/DeleteMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMap-request)))
  "Returns string type for a service object of type 'DeleteMap-request"
  "kortex_driver/DeleteMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteMap-request>)))
  "Returns md5sum for a message object of type '<DeleteMap-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteMap-request)))
  "Returns md5sum for a message object of type 'DeleteMap-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteMap-request>)))
  "Returns full string definition for message of type '<DeleteMap-request>"
  (cl:format cl:nil "MapHandle input~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteMap-request)))
  "Returns full string definition for message of type 'DeleteMap-request"
  (cl:format cl:nil "MapHandle input~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteMap-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteMap-response.msg.html

(cl:defclass <DeleteMap-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteMap-response (<DeleteMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteMap-response> is deprecated: use kortex_driver-srv:DeleteMap-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteMap-response>) ostream)
  "Serializes a message object of type '<DeleteMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteMap-response>) istream)
  "Deserializes a message object of type '<DeleteMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteMap-response>)))
  "Returns string type for a service object of type '<DeleteMap-response>"
  "kortex_driver/DeleteMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMap-response)))
  "Returns string type for a service object of type 'DeleteMap-response"
  "kortex_driver/DeleteMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteMap-response>)))
  "Returns md5sum for a message object of type '<DeleteMap-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteMap-response)))
  "Returns md5sum for a message object of type 'DeleteMap-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteMap-response>)))
  "Returns full string definition for message of type '<DeleteMap-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteMap-response)))
  "Returns full string definition for message of type 'DeleteMap-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteMap-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteMap)))
  'DeleteMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteMap)))
  'DeleteMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMap)))
  "Returns string type for a service object of type '<DeleteMap>"
  "kortex_driver/DeleteMap")