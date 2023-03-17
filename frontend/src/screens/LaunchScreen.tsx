import styled from 'styled-components/native';
import {ImageBackground} from 'react-native';

const LaunchScreen = () => {
  return (
    <Container source={require('../assets/images/launch_screen_large.png')}>
      <Title>My App</Title>
    </Container>
  );
};

const Container = styled(ImageBackground)`
  flex: 1;
  align-items: center;
  justify-content: center;
  background-color: white;
`;

const Title = styled.Text`
  font-size: 24px;
  font-weight: bold;
  color: white;
`;

export default LaunchScreen;
