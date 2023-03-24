import {StyleSheet, View, ImageBackground} from 'react-native';
import LoginForm from '../components/LoginForm';
import styled from 'styled-components/native';

const LoginScreen = () => {
  return (
    <Container source={require('../assets/images/login_screen.png')}>
      <View style={styles.container}>
        <LoginForm />
      </View>
    </Container>
  );
};

const Container = styled(ImageBackground)`
  flex: 1;
  padding-top: 40%;
  align-items: center;
  // justify-content: center;
  background-color: white;
`;

const styles = StyleSheet.create({
  container: {
    width: '80%',
    justifyContent: 'center',
    paddingHorizontal: 10,
  },
});

export default LoginScreen;
