# Returns the hierachy as one string from the first remaining part on.
def nextname(hierachy, last_processed_index):
    try:
        result = "".join(map(str, hierachy[last_processed_index+1:]))
    except Exception as e:
        rospy.logerr("Error encountered ", e)
    return result