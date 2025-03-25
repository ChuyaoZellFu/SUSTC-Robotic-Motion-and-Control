; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadAllDevices-request.msg.html

(cl:defclass <ReadAllDevices-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ReadAllDevices-request (<ReadAllDevices-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllDevices-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllDevices-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllDevices-request> is deprecated: use kortex_driver-srv:ReadAllDevices-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadAllDevices-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllDevices-request>) ostream)
  "Serializes a message object of type '<ReadAllDevices-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllDevices-request>) istream)
  "Deserializes a message object of type '<ReadAllDevices-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllDevices-request>)))
  "Returns string type for a service object of type '<ReadAllDevices-request>"
  "kortex_driver/ReadAllDevicesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllDevices-request)))
  "Returns string type for a service object of type 'ReadAllDevices-request"
  "kortex_driver/ReadAllDevicesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllDevices-request>)))
  "Returns md5sum for a message object of type '<ReadAllDevices-request>"
  "9f8a3530be8da22ec93eee7ab47ef9a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllDevices-request)))
  "Returns md5sum for a message object of type 'ReadAllDevices-request"
  "9f8a3530be8da22ec93eee7ab47ef9a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllDevices-request>)))
  "Returns full string definition for message of type '<ReadAllDevices-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllDevices-request)))
  "Returns full string definition for message of type 'ReadAllDevices-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllDevices-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllDevices-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllDevices-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadAllDevices-response.msg.html

(cl:defclass <ReadAllDevices-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:DeviceHandles
    :initform (cl:make-instance 'kortex_driver-msg:DeviceHandles)))
)

(cl:defclass ReadAllDevices-response (<ReadAllDevices-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllDevices-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllDevices-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllDevices-response> is deprecated: use kortex_driver-srv:ReadAllDevices-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadAllDevices-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllDevices-response>) ostream)
  "Serializes a message object of type '<ReadAllDevices-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllDevices-response>) istream)
  "Deserializes a message object of type '<ReadAllDevices-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllDevices-response>)))
  "Returns string type for a service object of type '<ReadAllDevices-response>"
  "kortex_driver/ReadAllDevicesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllDevices-response)))
  "Returns string type for a service object of type 'ReadAllDevices-response"
  "kortex_driver/ReadAllDevicesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllDevices-response>)))
  "Returns md5sum for a message object of type '<ReadAllDevices-response>"
  "9f8a3530be8da22ec93eee7ab47ef9a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllDevices-response)))
  "Returns md5sum for a message object of type 'ReadAllDevices-response"
  "9f8a3530be8da22ec93eee7ab47ef9a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllDevices-response>)))
  "Returns full string definition for message of type '<ReadAllDevices-response>"
  (cl:format cl:nil "DeviceHandles output~%~%================================================================================~%MSG: kortex_driver/DeviceHandles~%~%DeviceHandle[] device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllDevices-response)))
  "Returns full string definition for message of type 'ReadAllDevices-response"
  (cl:format cl:nil "DeviceHandles output~%~%================================================================================~%MSG: kortex_driver/DeviceHandles~%~%DeviceHandle[] device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllDevices-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllDevices-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllDevices-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadAllDevices)))
  'ReadAllDevices-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadAllDevices)))
  'ReadAllDevices-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllDevices)))
  "Returns string type for a service object of type '<ReadAllDevices>"
  "kortex_driver/ReadAllDevices")