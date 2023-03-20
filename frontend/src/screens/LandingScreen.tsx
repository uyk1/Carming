import {ImageBackground, View, Button} from 'react-native';
import styled from 'styled-components/native';

const LandingScreen = () => {
  return (
    <Container source={require('../assets/images/launch_screen_large.png')}>
      <LaunchView>
        <Title> 새로운 여정 경험을 선사하는</Title>
        <Title> 자율주행 서비스, 카밍 </Title>
        <BtnView>
          <LandingScreenBtn title="Hello" />
          <LandingScreenBtn title="Hello" />
        </BtnView>
      </LaunchView>
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
  font-size: 27px;
  font-weight: bold;
  color: white;
  text-align: center;
  text-shadow: 3px 3px 20px gray;
`;

const LaunchView = styled.View`
  height: 75%;
  width: 80%;
  display: flex;
`;

const BtnView = styled.View`
  display: flex;
`;

const LandingScreenBtn = styled.Button`
  background: brown;
`;

export default LandingScreen;
