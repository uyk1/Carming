import {Text} from 'react-native';
import styled from 'styled-components/native';

function HomeScreen() {
  return (
    <StyledSafeAreaView>
      <Text>HomeScreen</Text>
    </StyledSafeAreaView>
  );
}

const StyledSafeAreaView = styled.SafeAreaView`
  flex: 1;
  justify-content: center;
  align-items: center;
`;

export default HomeScreen;
