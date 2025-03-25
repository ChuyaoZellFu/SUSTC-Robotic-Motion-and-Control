; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UserProfile.msg.html

(cl:defclass <UserProfile> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (username
    :reader username
    :initarg :username
    :type cl:string
    :initform "")
   (firstname
    :reader firstname
    :initarg :firstname
    :type cl:string
    :initform "")
   (lastname
    :reader lastname
    :initarg :lastname
    :type cl:string
    :initform "")
   (application_data
    :reader application_data
    :initarg :application_data
    :type cl:string
    :initform ""))
)

(cl:defclass UserProfile (<UserProfile>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserProfile>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserProfile)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UserProfile> is deprecated: use kortex_driver-msg:UserProfile instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <UserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'username-val :lambda-list '(m))
(cl:defmethod username-val ((m <UserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:username-val is deprecated.  Use kortex_driver-msg:username instead.")
  (username m))

(cl:ensure-generic-function 'firstname-val :lambda-list '(m))
(cl:defmethod firstname-val ((m <UserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:firstname-val is deprecated.  Use kortex_driver-msg:firstname instead.")
  (firstname m))

(cl:ensure-generic-function 'lastname-val :lambda-list '(m))
(cl:defmethod lastname-val ((m <UserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:lastname-val is deprecated.  Use kortex_driver-msg:lastname instead.")
  (lastname m))

(cl:ensure-generic-function 'application_data-val :lambda-list '(m))
(cl:defmethod application_data-val ((m <UserProfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:application_data-val is deprecated.  Use kortex_driver-msg:application_data instead.")
  (application_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserProfile>) ostream)
  "Serializes a message object of type '<UserProfile>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'username))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'username))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'firstname))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'firstname))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lastname))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lastname))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'application_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'application_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserProfile>) istream)
  "Deserializes a message object of type '<UserProfile>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'username) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'username) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firstname) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'firstname) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lastname) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lastname) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'application_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'application_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserProfile>)))
  "Returns string type for a message object of type '<UserProfile>"
  "kortex_driver/UserProfile")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserProfile)))
  "Returns string type for a message object of type 'UserProfile"
  "kortex_driver/UserProfile")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserProfile>)))
  "Returns md5sum for a message object of type '<UserProfile>"
  "4599942b2b24021432dda0e643774952")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserProfile)))
  "Returns md5sum for a message object of type 'UserProfile"
  "4599942b2b24021432dda0e643774952")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserProfile>)))
  "Returns full string definition for message of type '<UserProfile>"
  (cl:format cl:nil "~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserProfile)))
  "Returns full string definition for message of type 'UserProfile"
  (cl:format cl:nil "~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserProfile>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'username))
     4 (cl:length (cl:slot-value msg 'firstname))
     4 (cl:length (cl:slot-value msg 'lastname))
     4 (cl:length (cl:slot-value msg 'application_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserProfile>))
  "Converts a ROS message object to a list"
  (cl:list 'UserProfile
    (cl:cons ':handle (handle msg))
    (cl:cons ':username (username msg))
    (cl:cons ':firstname (firstname msg))
    (cl:cons ':lastname (lastname msg))
    (cl:cons ':application_data (application_data msg))
))
