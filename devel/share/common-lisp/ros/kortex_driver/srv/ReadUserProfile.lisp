; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadUserProfile-request.msg.html

(cl:defclass <ReadUserProfile-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle)))
)

(cl:defclass ReadUserProfile-request (<ReadUserProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadUserProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadUserProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadUserProfile-request> is deprecated: use kortex_driver-srv:ReadUserProfile-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadUserProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadUserProfile-request>) ostream)
  "Serializes a message object of type '<ReadUserProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadUserProfile-request>) istream)
  "Deserializes a message object of type '<ReadUserProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadUserProfile-request>)))
  "Returns string type for a service object of type '<ReadUserProfile-request>"
  "kortex_driver/ReadUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadUserProfile-request)))
  "Returns string type for a service object of type 'ReadUserProfile-request"
  "kortex_driver/ReadUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadUserProfile-request>)))
  "Returns md5sum for a message object of type '<ReadUserProfile-request>"
  "88ed6f69b3ce05a4b3c054859c8deeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadUserProfile-request)))
  "Returns md5sum for a message object of type 'ReadUserProfile-request"
  "88ed6f69b3ce05a4b3c054859c8deeff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadUserProfile-request>)))
  "Returns full string definition for message of type '<ReadUserProfile-request>"
  (cl:format cl:nil "UserProfileHandle input~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadUserProfile-request)))
  "Returns full string definition for message of type 'ReadUserProfile-request"
  (cl:format cl:nil "UserProfileHandle input~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadUserProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadUserProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadUserProfile-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadUserProfile-response.msg.html

(cl:defclass <ReadUserProfile-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:UserProfile
    :initform (cl:make-instance 'kortex_driver-msg:UserProfile)))
)

(cl:defclass ReadUserProfile-response (<ReadUserProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadUserProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadUserProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadUserProfile-response> is deprecated: use kortex_driver-srv:ReadUserProfile-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadUserProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadUserProfile-response>) ostream)
  "Serializes a message object of type '<ReadUserProfile-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadUserProfile-response>) istream)
  "Deserializes a message object of type '<ReadUserProfile-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadUserProfile-response>)))
  "Returns string type for a service object of type '<ReadUserProfile-response>"
  "kortex_driver/ReadUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadUserProfile-response)))
  "Returns string type for a service object of type 'ReadUserProfile-response"
  "kortex_driver/ReadUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadUserProfile-response>)))
  "Returns md5sum for a message object of type '<ReadUserProfile-response>"
  "88ed6f69b3ce05a4b3c054859c8deeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadUserProfile-response)))
  "Returns md5sum for a message object of type 'ReadUserProfile-response"
  "88ed6f69b3ce05a4b3c054859c8deeff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadUserProfile-response>)))
  "Returns full string definition for message of type '<ReadUserProfile-response>"
  (cl:format cl:nil "UserProfile output~%~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadUserProfile-response)))
  "Returns full string definition for message of type 'ReadUserProfile-response"
  (cl:format cl:nil "UserProfile output~%~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadUserProfile-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadUserProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadUserProfile-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadUserProfile)))
  'ReadUserProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadUserProfile)))
  'ReadUserProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadUserProfile)))
  "Returns string type for a service object of type '<ReadUserProfile>"
  "kortex_driver/ReadUserProfile")