; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude CreateUserProfile-request.msg.html

(cl:defclass <CreateUserProfile-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:FullUserProfile
    :initform (cl:make-instance 'kortex_driver-msg:FullUserProfile)))
)

(cl:defclass CreateUserProfile-request (<CreateUserProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateUserProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateUserProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateUserProfile-request> is deprecated: use kortex_driver-srv:CreateUserProfile-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <CreateUserProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateUserProfile-request>) ostream)
  "Serializes a message object of type '<CreateUserProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateUserProfile-request>) istream)
  "Deserializes a message object of type '<CreateUserProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateUserProfile-request>)))
  "Returns string type for a service object of type '<CreateUserProfile-request>"
  "kortex_driver/CreateUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateUserProfile-request)))
  "Returns string type for a service object of type 'CreateUserProfile-request"
  "kortex_driver/CreateUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateUserProfile-request>)))
  "Returns md5sum for a message object of type '<CreateUserProfile-request>"
  "406efcad037e6ae1bf367e1fe039a21c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateUserProfile-request)))
  "Returns md5sum for a message object of type 'CreateUserProfile-request"
  "406efcad037e6ae1bf367e1fe039a21c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateUserProfile-request>)))
  "Returns full string definition for message of type '<CreateUserProfile-request>"
  (cl:format cl:nil "FullUserProfile input~%~%================================================================================~%MSG: kortex_driver/FullUserProfile~%~%UserProfile user_profile~%string password~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateUserProfile-request)))
  "Returns full string definition for message of type 'CreateUserProfile-request"
  (cl:format cl:nil "FullUserProfile input~%~%================================================================================~%MSG: kortex_driver/FullUserProfile~%~%UserProfile user_profile~%string password~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateUserProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateUserProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateUserProfile-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude CreateUserProfile-response.msg.html

(cl:defclass <CreateUserProfile-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle)))
)

(cl:defclass CreateUserProfile-response (<CreateUserProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateUserProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateUserProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateUserProfile-response> is deprecated: use kortex_driver-srv:CreateUserProfile-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <CreateUserProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateUserProfile-response>) ostream)
  "Serializes a message object of type '<CreateUserProfile-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateUserProfile-response>) istream)
  "Deserializes a message object of type '<CreateUserProfile-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateUserProfile-response>)))
  "Returns string type for a service object of type '<CreateUserProfile-response>"
  "kortex_driver/CreateUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateUserProfile-response)))
  "Returns string type for a service object of type 'CreateUserProfile-response"
  "kortex_driver/CreateUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateUserProfile-response>)))
  "Returns md5sum for a message object of type '<CreateUserProfile-response>"
  "406efcad037e6ae1bf367e1fe039a21c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateUserProfile-response)))
  "Returns md5sum for a message object of type 'CreateUserProfile-response"
  "406efcad037e6ae1bf367e1fe039a21c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateUserProfile-response>)))
  "Returns full string definition for message of type '<CreateUserProfile-response>"
  (cl:format cl:nil "UserProfileHandle output~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateUserProfile-response)))
  "Returns full string definition for message of type 'CreateUserProfile-response"
  (cl:format cl:nil "UserProfileHandle output~%~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateUserProfile-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateUserProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateUserProfile-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateUserProfile)))
  'CreateUserProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateUserProfile)))
  'CreateUserProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateUserProfile)))
  "Returns string type for a service object of type '<CreateUserProfile>"
  "kortex_driver/CreateUserProfile")