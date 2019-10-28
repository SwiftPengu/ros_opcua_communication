import string

ACCEPT_ALL = lambda topicName: True

'''
    Returns a filtering function, which returns True iff a topic string should be included
        If filters are specified, then the filters specify the list of topics to include, with the topics to remove excluded.

        Filters are defined as a semicolon-separated list of ROS topic tree parts
        Topic parts can be prepended by '+' (default) or '-' to specify inclusion or exclusion respectively.
        Priority is assigned by the definition order.

        
        EBNF:

        filters := (filter ';' )* ;
        filter := ( '+' | '-' )? topic ;
        topic := '/' | ('/' IDENTIFIER )+ ;
        IDENTIFIER := [A-Za-z0-9]+ ;


        Examples:
        '+/;-/foo' Include everything, except everything under /foo
        '+/foo' Include everything in /foo (and nothing more)
        '/foo' Include everything in /foo (and nothing more)
'''
def parseFilters(filters):    
    if filters is None:
        return ACCEPT_ALL

    # Split and filter empty elements
    parsedFilters = filter(None, [f.strip() for f in filters.split(';')])
    def isIncluded(topicName):
        result = False
        # Go over the filter list
        for parsedFilter in parsedFilters:
            rawFilter = string.lstrip(parsedFilter, chars='-+')
            
            # If we have a match
            if topicName.startswith(rawFilter):
                # Exclude if negative (with stop below), else include
                result = not parsedFilter.startswith('-')

            # Stop on failure
            if result == False:
                break
        return result
    return lambda topicName: isIncluded(topicName)

if __name__ == '__main__':
    print('Running Tests')
    testQueries = ['/', '/bar', '/foo', '/foo/bar', '/foo/foo', '/bar/bar', '/bar/foo', '/foo/bar/foo']

    def test(testFilter, testAnswers):
        testExec = [parseFilters(testFilter)(q) for q in testQueries]
        try:
            assert(testExec == testAnswers)
        except AssertionError as e:
            print('Test', testFilter)
            print('Expected', testAnswers)
            print('Actual', testExec)
            raise e

    test('/foo', testAnswers=[False, False, True, True, True, False, False, True])
    test('+/', testAnswers=[True, True, True, True, True, True, True, True])
    test('-/', testAnswers=[False, False, False, False, False, False, False, False])
    test(None, testAnswers=[True, True, True, True, True, True, True, True])
    test('/foo;-/foo/bar', testAnswers=[False, False, True, False, True, False, False, False])
    # Priority test, last filter does nothing (exclusion takes priority)
    test('/foo;-/foo/bar;+/foo/bar/foo', testAnswers=[False, False, True, False, True, False, False, False])
    # Test empty filters
    test('/foo;;+/foo/bar/foo', testAnswers=[False, False, True, True, True, False, False, True])