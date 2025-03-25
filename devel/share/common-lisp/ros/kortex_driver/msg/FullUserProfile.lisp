; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FullUserProfile.msg.html

(cl:defclass <FullUserProfile> (roslisp-msg-protocol:ros-message)
  ((user_profile
    :reader user_profile
    :initarg :user_profile
    :type kortex_driver-msg:UserProfile
    :initform (cl:make-instance 'kortex_driver-msg:UserProfile))
   (password
    :reader password
    :initarg :password
    :type cl:string
    :initform ""))
)

(cl:defclass FullUserProfile (<FullUserProfile>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullUserProfile>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullUserProfile)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FullUserProfile> is deprecated: use kortex_driver-msg:FullUserProfile instead.")))

(cl:ensure-generic-function 'user_profile-val :lambda-list '(m))
(cl:defmethod user_profile-val ((m <FullUserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_profile-val is deprecated.  Use kortex_driver-msg:user_profile instead.")
  (user_profile m))

(cl:ensure-generic-function 'password-val :lambda-list '(m))
(cl:defmethod password-val ((m <FullUserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:password-val is deprecated.  Use kortex_driver-msg:password instead.")
  (password m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullUserProfile>) ostream)
  "Serializes a message object of type '<FullUserProfile>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_profile) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'password))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'password))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullUserProfile>) istream)
  "Deserializes a message object of type '<FullUserProfile>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_profile) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'password) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'password) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullUserProfile>)))
  "Returns string type for a message object of type '<FullUserProfile>"
  "kortex_driver/FullUserProfile")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullUserProfile)))
  "Returns string type for a message object of type 'FullUserProfile"
  "kortex_driver/FullUserProfile")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullUserProfile>)))
  "Returns md5sum for a message object of type '<FullUserProfile>"
  "5832f6aa4b0c784ce33a85ff505da582")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullUserProfile)))
  "Returns md5sum for a message object of type 'FullUserProfile"
  "5832f6aa4b0c784ce33a85ff505da582")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullUserProfile>)))
  "Returns full string definition for message of type '<FullUserProfile>"
  (cl:format cl:nil "~%UserProfile user_profile~%string password~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullUserProfile)))
  "Returns full string definition for message of type 'FullUserProfile"
  (cl:format cl:nil "~%UserProfile user_profile~%string password~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullUserProfile>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_profile))
     4 (cl:length (cl:slot-value msg 'password))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullUserProfile>))
  "Converts a ROS message object to a list"
  (cl:list 'FullUserProfile
    (cl:cons ':user_profile (user_profile msg))
    (cl:cons ':password (password msg))
))
