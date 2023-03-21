import {ImageBackground} from 'react-native';
import styled from 'styled-components/native';

const LandingScreen = () => {
  return (
    <Container source={require('../assets/images/launch_screen_large.png')}>
      <LaunchView>
        <Title> 새로운 여정 경험을 선사하는 </Title>
        <Title> 자율주행 서비스, 카밍 </Title>
        <BtnView>
          <LandingScreenBtn title="로그인" />
          <LandingScreenBtn title="회원가입" />
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
  font-family: 'SeoulNamsanEB';
  font-size: 27px;
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
  background-color: blue;
  padding: 10px;
  border-radius: 5px;
`;

export default LandingScreen;
